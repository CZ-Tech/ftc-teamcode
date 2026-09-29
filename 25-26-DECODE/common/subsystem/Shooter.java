package org.firstinspires.ftc.teamcode.common.subsystem;

import static org.firstinspires.ftc.teamcode.common.Globals.SHOOT_DIS_OFFSET_MM;
import static org.firstinspires.ftc.teamcode.common.Globals.SHOOTER_K_LINEAR;
import static org.firstinspires.ftc.teamcode.common.Globals.SHOOTER_B_LINEAR;
import static org.firstinspires.ftc.teamcode.common.Globals.ShooterD;
import static org.firstinspires.ftc.teamcode.common.Globals.ShooterI;
import static org.firstinspires.ftc.teamcode.common.Globals.ShooterP;
import static org.firstinspires.ftc.teamcode.common.Globals.ShooterF;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

import java.util.Arrays;

@Config
public class Shooter {
    private Robot robot;
    public DcMotorEx left, right;
    public static double RPM = 3500 ;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    public double r = 0;
    private boolean isRunning = false;

    public static int AccelFilterWindowSize = 10;
    public double[] shootAccelWindow = new double[AccelFilterWindowSize];
    private int currentShootAccelWindowIndex = 0;
    private double lastCallAccelTime = System.currentTimeMillis();
    private double lastVelocity = 0;

    // 非线性电压补偿：高电压段球速过快需大幅降速，低电压段球速不足需适当补速
    public static double kneeVoltage = 12.6;   // 拐点电压
    public static double highGain = 0.04;       // 高电压段每0.1V降速比例
    public static double lowGain  = 0.005;       // 低电压段每0.1V补速比例

    private double getRPM(double x){
        return distanceToRPM(DistanceUnit.MM, x);
    }

    /**
     *
     * @param robot
     */
    public Shooter(Robot robot) {
        this.robot = robot;
        left = robot.hardwareMap.get(DcMotorEx.class, Globals.leftShooter);
        right = robot.hardwareMap.get(DcMotorEx.class, Globals.rightShooter);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDF();

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void updatePIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(ShooterP, ShooterI, ShooterD, ShooterF);
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public Shooter back(){
        isRunning = false;
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setPower(-0.5);
        right.setPower(0.5);
        return this;
    }

    /**
     * Gets the voltage compensation gain factor.
     * When actual battery voltage drops below nominal, target RPM is boosted proportionally.
     * @return gain factor
     */
    public double getVoltageGain() {
        double actualVoltage = robot.getVoltage();
        robot.telemetry.addData("BatteryVoltage", actualVoltage);

        double delta = actualVoltage - kneeVoltage; // >0偏高需降速, <0偏低需补速
        if (delta > 0) {
            return 1.0 - delta * highGain;
        } else {
            return 1.0 - delta * lowGain;    // delta为负，减负即加正
        }
    }

    private void setTargetVelocity(double leftRPM, double rightRPM) {
        isRunning = true;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMotorEnable();
        right.setMotorEnable();
        left.setPower(1.0);
        right.setPower(1.0);
        left.setVelocity(leftRPM / 60.0 * COUNTS_PER_MOTOR_REV);
        right.setVelocity(rightRPM / 60.0 * COUNTS_PER_MOTOR_REV);
        robot.telemetry.addData("targetRPM", r);
    }

    public Shooter shoot(double distanceMM) {
        r = getRPM(distanceMM);
        double gain = getVoltageGain();
        setTargetVelocity(r * gain, r * gain);
        return this;
    }

    /**
     * This method only accelerates the shooter to the target RPM, without actually ejecting the ring.
     * @param targetRPM Target RPM
     * @return Chained call
     */
    public Shooter staticShoot(double targetRPM) {
//        updatePIDF();
        robot.telemetry.addLine(ShooterP + " " + ShooterI + " " + ShooterD + " " + ShooterF);
        double gain = getVoltageGain();
        r = targetRPM * gain;
        setTargetVelocity(r, r);
        return this;
    }

    public Shooter shoot(int i, double x1, double x2) {
        setTargetVelocity(x1, x2);
        return this;
    }

    public Shooter shoot() {
        return this.staticShoot(RPM);
    }

    public Shooter stop() {
        isRunning = false;
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setPower(0);
        right.setPower(0);
        return this;
    }

    public static double distanceToRPM(DistanceUnit unit, double distance) {
        double x = DistanceUnit.MM.fromUnit(unit, distance);
        if (x <= 0) return 3100;
        x += SHOOT_DIS_OFFSET_MM;
        // Linear compensation function that decreases as distance increases: f(x) = k*x + b
        // where k is SHOOTER_K_LINEAR (expected negative), b is SHOOTER_B_LINEAR (expected positive)
        double linearComp = SHOOTER_K_LINEAR * x + SHOOTER_B_LINEAR;
        // return 4e-08 * x * x * x - 0.0005 * x * x + 2.059 * x + 681.3 + MathUtils.clamp(linearComp, -120, Double.MAX_VALUE);

//        return -7.7005e-14 * Math.pow(x, 5)
//                + 1.3321e-9 * Math.pow(x, 4)
//                - 9.1296e-6 * Math.pow(x, 3)
//                + 3.1007e-2 * Math.pow(x, 2)
//                - 5.1869e1 * x
//                + 3.6976e4;
//        return -1.7091E-13 * Math.pow(x,5) + 2.8438E-9 * Math.pow(x,4) - 1.8642E-5 * Math.pow(x,3) + 6.0178E-2 * Math.pow(x,2) - 9.5354E1 * x + 6.2096E4;
        return (5.2250e-5 * x * x + 5.9661e-2 * x + 2.5873e3) * 0.95 * (0.00004 * x + 1);
    }

    public double getShooterAccel() {
        double currentVelocity = (left.getVelocity() + right.getVelocity()) / 2.0 / COUNTS_PER_MOTOR_REV * 60.0;
        double currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastCallAccelTime) / 1000.0;

        double accel = 0;
        if (dt > 0) {
            accel = (currentVelocity - lastVelocity) / dt;
        }

        lastVelocity = currentVelocity;

        double totalAccel = 0.0;
        if (currentTime - lastCallAccelTime >= 500) {
            Arrays.fill(shootAccelWindow, 0.0D);
        } else {
            for (int i = 0; i < shootAccelWindow.length; i++) {
                totalAccel += shootAccelWindow[i];
            }
        }
        lastCallAccelTime = currentTime;

        shootAccelWindow[currentShootAccelWindowIndex] = accel;
        currentShootAccelWindowIndex++;
        if (currentShootAccelWindowIndex >= shootAccelWindow.length-1) {
            currentShootAccelWindowIndex = 0;
        }
        return totalAccel / shootAccelWindow.length;
    }

    public boolean isPrepared() {
        double leftRPM = left.getVelocity() / COUNTS_PER_MOTOR_REV * 60.0;
        double rightRPM = right.getVelocity() / COUNTS_PER_MOTOR_REV * 60.0;

        robot.telemetry.addData("ShootSpeedOff", Math.abs(leftRPM - r));
        robot.telemetry.addData("leftRPM", leftRPM);
        robot.telemetry.addData("rightRPM", rightRPM);

        return Math.abs(leftRPM - r) <= Globals.SHOOT_VEL_TOLERANCE &&
                Math.abs(rightRPM - r) <= Globals.SHOOT_VEL_TOLERANCE &&
                isRunning;
    }

    public void setPower(double power) {
        isRunning = false;
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setPower(power);
        right.setPower(power);
    }
}
