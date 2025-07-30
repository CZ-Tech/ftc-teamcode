package org.firstinspires.ftc.teamcode.common.subsystem;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

import org.firstinspires.ftc.teamcode.common.Robot;
public class Thrower {

    private final Robot robot;
    public DcMotorEx Liftmotor;

    public Thrower(Robot robot) {
        this.robot = robot;
        Liftmotor=robot.hardwareMap.get(DcMotorEx.class,"LiftMotor");//eh 2
        Liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Thrower Lift(){
        Liftmotor.setPower(1.0);
        Liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.waitFor(1000);
        Stop();
        return this;
    }

    public Thrower Lift(Double power){
        Liftmotor.setPower(power);
        Liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return this;
    }

    public Thrower Down(){
        Liftmotor.setPower(-1.0);
        robot.waitFor(1000);
        Stop();
        return this;
    }

    public Thrower Down(double power){
        Liftmotor.setPower(power);
        return this;
    }

    public Thrower Stop(){
        Liftmotor.setPower(0.0);
        Liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return this;
    }
    public Thrower giveaway(){
        runLiftMotor(600,1.0);
        return this;
    }

    public Thrower runLiftMotor(int movecounts, double power) {
        Liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Liftmotor.setTargetPosition(Liftmotor.getCurrentPosition() + movecounts);
        Liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Liftmotor.setPower(power);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (robot.opMode.opModeIsActive() && Liftmotor.isBusy() && runtime.milliseconds() <= 1500) ;
        Liftmotor.setPower(0);
        Liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return this;
    }

}
