package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Intaker {
    public final Robot robot;

    public CRServoImplEx leftArm, rightArm;
    public DcMotorEx intakerItself;

    public Intaker(Robot robot) {
        this.robot = robot;

//        leftArm = robot.hardwareMap.get(CRServoImplEx.class, Globals.leftArm);
//        rightArm = robot.hardwareMap.get(CRServoImplEx.class, Globals.rightArm);
//
//        leftArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        rightArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intakerItself = robot.hardwareMap.get(DcMotorEx.class, Globals.intaker);

        this.setArmPower(0);
    }

    private Intaker setArmPower(double power){
        intakerItself.setPower(power);
        return this;
    }

    public Intaker intakerIn(double power){
        return this.setArmPower(power);
    }

    public Intaker intakerIn(){
        return this.intakerIn(1);
    }

    public Intaker intakerArmStop(){
        return this.setArmPower(0);
    }
}
