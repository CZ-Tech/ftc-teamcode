package org.firstinspires.ftc.teamcode.common.subsystem;


import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Robot;

//@Confg
public class Claw {
    public ServoImplEx claw;
    public ServoImplEx rightturning;
    public ServoImplEx clawturning;
    private Robot robot;

    public static double UP = 800;
    public static double PARALLE = 1660;
    public static double DOWN = 2400;
    public static double GRAB = 880;
    public static double RELEASE = 2000;

    public double currPos;

    public static double calcTurningPos(double x){
        return 50.0 / 9.0 * x + 1500;
    }

    public Claw(Robot robot){
        this.robot = robot;
        claw = robot.hardwareMap.get(ServoImplEx.class, "claw");
        rightturning = robot.hardwareMap.get(ServoImplEx.class, "rightturning");
        clawturning = robot.hardwareMap.get(ServoImplEx.class, "clawturning");
        clawturning.setPwmRange(new PwmControl.PwmRange(500, 2500));
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightturning.setPwmRange(new PwmControl.PwmRange(500, 2500));

//        claw.setPwmEnable();
//        clawturning.setPwmEnable();

//        clawturning.setPosition(calcTurningPos(0));
        currPos = 0;
    }

    public Claw up(){
        rightturning.setPosition((UP - 500.0) / 2000.0);
        return this;
    }

    public Claw down(){
        rightturning.setPosition((DOWN - 500.0) / 2000.0);
        return this;
    }

    public Claw paralle(){
        rightturning.setPosition((PARALLE - 500.0) / 2000.0);
        return this;
    }

    public Claw grab(){
        claw.setPosition((GRAB - 500.0) / 2000.0);
        return this;
    }

    public Claw release(){
        claw.setPosition((RELEASE - 500.0) / 2000.0);
        return this;
    }
}
