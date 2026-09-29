package org.firstinspires.ftc.teamcode.common.subsystem;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Belt {
    public Robot robot;
    public CRServoImplEx leftBelt, rightBelt;
    public DcMotorEx beltItself;

    public Belt(Robot robot){
        this.robot = robot;

//        leftBelt = robot.hardwareMap.get(CRServoImplEx.class, Globals.leftBelt);
//        rightBelt = robot.hardwareMap.get(CRServoImplEx.class, Globals.rightBelt);
//
//        leftBelt.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        rightBelt.setPwmRange(new PwmControl.PwmRange(500, 2500));

        beltItself = robot.hardwareMap.get(DcMotorEx.class, Globals.belt);
    }

    private Belt beltPower(double power){
        beltItself.setPower(power);
        return this;
    }

    public Belt up(double power){
        return beltPower(Math.abs(power));
    }

    public Belt up(){
        return this.up(-1);
    }

    public Belt down(){
        return beltPower(-0.8);
    }

    public Belt stop(){
        return this.beltPower(0);
    }
}
