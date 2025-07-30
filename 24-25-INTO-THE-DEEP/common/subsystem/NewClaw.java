package org.firstinspires.ftc.teamcode.common.subsystem;


import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;


public class NewClaw {
    public ServoImplEx claw;
    public ServoImplEx rightturning;
    public ServoImplEx clawturning;
    private Robot robot;
    public double currPos;

    public enum Pos {
        GRAB(1250),
        RELEASE(2112),
        UP(1325),// 1460
        DOWN(2188),// 2323
        OBSERVER(1820),//1955
        CENTER(1500),
        LEFT(1750),
        RIGHT(1250);


        public final double position;

        Pos(double position) {
            this.position = position;
        }

        public double getPosition() {
            return (this.position - 500.0) / 2000.0;
        }
    }

    public NewClaw(Robot robot) {
        this.robot = robot;
        claw = robot.hardwareMap.get(ServoImplEx.class, "claw");
        rightturning = robot.hardwareMap.get(ServoImplEx.class, "rightturning");
        clawturning = robot.hardwareMap.get(ServoImplEx.class, "clawturning");
        clawturning.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightturning.setPwmRange(new PwmControl.PwmRange(500, 2500));

        currPos = 0;
    }

    public NewClaw up() {
        rightturning.setPosition(Pos.UP.getPosition());
        return this;
    }

    public NewClaw down() {
//        rightturning.setPosition((Globals.VAR - 500.0) / 2000.0);
        rightturning.setPosition(Pos.DOWN.getPosition());
        return this;
    }

    public NewClaw turnRight() {
        clawturning.setPosition(Pos.RIGHT.getPosition());
        return this;
    }

    public NewClaw turnLeft() {
        clawturning.setPosition(Pos.LEFT.getPosition());
        return this;
    }

    public NewClaw turnTo(double angel) {
        clawturning.setPosition((((angel + 90) / 180 * 1000 + 1000) - 500) / 2000);
        return this;
    }

    public NewClaw observer() {
//        rightturning.setPosition((Globals.VAR1-500)/2000.0);
        rightturning.setPosition(Pos.OBSERVER.getPosition());
        clawturning.setPosition(Pos.CENTER.getPosition());
        robot.subsystem.stretcher.RightExpandBack();
        return this;
    }
    public NewClaw drop() {
        release();
        rightturning.setPosition(Pos.OBSERVER.getPosition());
        return this;
    }
    public NewClaw grab() {
        claw.setPosition(Pos.GRAB.getPosition());
        return this;
    }

    public NewClaw release() {
        claw.setPosition(Pos.RELEASE.getPosition());
        return this;
    }
}
