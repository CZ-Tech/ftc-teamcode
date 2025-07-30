package org.firstinspires.ftc.teamcode.common.subsystem;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

//@Confg
public class Arm {

    private final Robot robot;
    public DcMotorEx leftArmMotor;
    public DcMotorEx rightArmMotor;
    public DcMotorEx oneShot;

    public static int Right_TOP_POS = -1722;
    public static int Left_TOP_POS = 1734;
    public static int Right_READY_POS = -1165;
    public static int Left_READY_POS = 1185;
    public static int Right_DUNK_POS = Right_READY_POS;//TODO 改
    public static int Left_DUNK_POS = Left_READY_POS;//TODO 改
    public static int RIGHT_END_POS = -1287;
    public static int LEFT_END_POS = 1259;
    public static int ONESHOT_READY = 6466;

    public static double MAX_POWER = 1;
    public static double MIN_POWER = 0.5;

    public Arm(Robot robot) {
        this.robot = robot;
        leftArmMotor = robot.hardwareMap.get(DcMotorEx.class, "leftArmMotor");//eh 0
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor = robot.hardwareMap.get(DcMotorEx.class, "rightArmMotor");//eh 1
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        oneShot = robot.hardwareMap.get(DcMotorEx.class, "oneShot");
        oneShot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        oneShot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oneShot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public Arm oneShot(){
        return this.oneShot(1);
    }

    public Arm oneShot(double power){
        oneShot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oneShot.setPower(power);
        return this;
    }
    public Arm oneShotReady(){
        oneShot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        oneShot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        oneShot.setTargetPosition(oneShot.getCurrentPosition() + ONESHOT_READY);
        oneShot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        oneShot.setPower(1);
        return this;
    }

    public Arm oneShotStop(){
        oneShot.setPower(0);
        return this;
    }

    //让机械臂向上伸展
    public Arm up(){
        return this.up(1);
    }

    public Arm up(double power) {
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmMotor.setPower(Math.abs(power));
        rightArmMotor.setPower(-Math.abs(power));
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return this;
    }
    //让机械臂向下
    public Arm down(){
        down(1);
        return this;
    }

    public Arm down(double power) {
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmMotor.setPower(-Math.abs(power));
        rightArmMotor.setPower(Math.abs(power));
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return this;
    }

    //让机械臂停止
    public Arm stop() {
        leftArmMotor.setPower(0.001);
        rightArmMotor.setPower(-0.001);

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return this;
    }

    public Arm EndTop(){
        return this.EndTop(1);
    }

    public Arm EndTop(double power){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runLeftMotor(LEFT_END_POS, Math.abs(power));
        runRightMotor(RIGHT_END_POS,-Math.abs(power));
        return this;
    }

    public Arm EndButtom(double power){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runLeftMotor(-LEFT_END_POS, -Math.abs(power));
        runRightMotor(-RIGHT_END_POS,Math.abs(power));
        return this;
    }
    //让机械臂到顶部

    public Arm ThrowTop(double power){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runLeftMotor(Left_TOP_POS, Math.abs(power));
        runRightMotor(Right_TOP_POS,-Math.abs(power));
        return this;
    }

    public Arm ThrowTop(){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runLeftMotor(Left_TOP_POS, 1);
        runRightMotor(Right_TOP_POS,-1);
        return this;
    }


    //让机械臂到底部
    public Arm ThrowBottom(double power){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runLeftMotor(-Left_TOP_POS, -Math.abs(power));
        runRightMotor(-Right_TOP_POS,Math.abs(power));
        return  this;
    }

    public Arm ThrowBottom(){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runLeftMotor(-Left_TOP_POS, -Math.abs(1));
        runRightMotor(-Right_TOP_POS,Math.abs(1));
        return  this;
    }

    public Arm DunkReady(){
        return DunkReady(1);
    }

    public Arm DunkReady(double power){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runRightMotor(Right_READY_POS,-Math.abs(power));
        runLeftMotor(Left_READY_POS,Math.abs(power));
        return this;
    }


    public Arm DunkTop(double power){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runRightMotor(Right_DUNK_POS,-Math.abs(power));
        runLeftMotor(Left_DUNK_POS,Math.abs(power));
        return this;
    }

    public Arm DunkTop(){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runRightMotor(Right_DUNK_POS,-1);
        runLeftMotor(Left_DUNK_POS,1);

        return this;
    }


    public Arm DunkBottom(double power){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runRightMotor(-Right_DUNK_POS,Math.abs(power));
        runLeftMotor(-Left_DUNK_POS,-Math.abs(power));
        return this;
    }

    public Arm DunkBottom(){
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runRightMotor(-Right_DUNK_POS,1);
        runLeftMotor(-Left_DUNK_POS,-1);
        return this;
    }

    public Arm runLeftMotor(int movecounts, double power){
        int targetPosition = leftArmMotor.getCurrentPosition() + movecounts;
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmMotor.setTargetPosition(leftArmMotor.getCurrentPosition() + movecounts);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmMotor.setPower(power);
//        ElapsedTime runtime = new ElapsedTime();
//        runtime.reset();
//        while (robot.opMode.opModeIsActive() && leftArmMotor.isBusy() && runtime.milliseconds() <= 1500){
//            double cPower = calculatePower(leftArmMotor.getCurrentPosition(), targetPosition);
//            leftArmMotor.setPower(cPower);
//        }
        return this;
    }
    public Arm runRightMotor(int movecounts, double power) {
        int targetPosition = rightArmMotor.getCurrentPosition() + movecounts;
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setTargetPosition(rightArmMotor.getCurrentPosition() + movecounts);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setPower(power);
//        ElapsedTime runtime = new ElapsedTime();
//        runtime.reset();
//        while (robot.opMode.opModeIsActive() && rightArmMotor.isBusy() && runtime.milliseconds() <= 1500){
//            double cPower = calculatePower(rightArmMotor.getCurrentPosition(), targetPosition);
//            rightArmMotor.setPower(cPower);
//        }
        return this;
    }

    private double calculatePower(int currentPosition, int targetPosition) {
        int distance = Math.abs(targetPosition - currentPosition);
        int totalDistance = Math.abs(targetPosition);

        // 线性插值计算功率
        double power = MIN_POWER + (MAX_POWER - MIN_POWER) * (distance / (double) totalDistance);

        // 确保功率在合理范围内
        return Math.min(Math.max(power, MIN_POWER), MAX_POWER);
    }
}
