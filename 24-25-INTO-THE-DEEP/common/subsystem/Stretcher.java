package org.firstinspires.ftc.teamcode.common.subsystem;


import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Robot;

//@Confg
public class Stretcher {
    public static double SPAN_POWER = -0.6;
    public static int SPAN_SLEEP = 500;

    private final Robot robot;
    public ServoImplEx rightexpand;
    public strectchStat stat;
    public ServoImplEx leftexpand;


    public static int rightback1 = 2175;
    public static int rightfront1 = 1575;

    public double rightback = (rightback1 - 500.0) / 2000;
    public double rightfront = (rightfront1 - 500.0) / 2000;

    public static int leftback1 = 850;
    public static int leftfront1 = 1460;

    public double leftback = (leftback1 - 500.0) / 2000;
    public double leftfront = (leftfront1 - 500.0) / 2000;

    public enum strectchStat {
        FORWARD,
        BACKWARD
    }

    /*
    爪子
      |
      |
    3-|
      |
      |
      |
    2-|
      |
      |
      |
    1-
     */


    public Stretcher(Robot robot) {
        this.robot = robot;
        rightexpand = robot.hardwareMap.get(ServoImplEx.class, "rightexpand");//eh 1
        rightexpand.setPwmRange(new PwmControl.PwmRange(500, 2500));
        leftexpand = robot.hardwareMap.get(ServoImplEx.class, "leftexpand");//ch 1
        leftexpand.setPwmRange(new PwmControl.PwmRange(500, 2500));

//        rightexpand.setPwmEnable();
//        leftexpand.setPwmEnable();
    }

    public enum bluePosition {
        INIT(2000),
        STOP(1850),
        RUN(1750);

        private double position;

        bluePosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return (position - 500.0) / 2000.0;
        }

        public void setPosition(double position) {
            this.position = position;
        }

        public void addPosition(double position) {
            this.position += position;
        }
    }

    public enum blackPosition {
        ;

        private double position;

        blackPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return (position - 500.0) / 2000.0;
        }

        public void setPosition(double position) {
            this.position = position;
        }

        public void addPosition(double position) {
            this.position += position;
        }
    }

    public void enableServo() {
        rightexpand.setPwmEnable();
    }

    public void disableServo() {
        rightexpand.setPwmDisable();
    }



    public Stretcher rightRun(){
        rightexpand.setPwmEnable();
        return this;
    }

    public Stretcher rightStop(){
        rightexpand.setPwmDisable();
        rightexpand.setPosition(rightexpand.getPosition());
        return this;
    }

    public Stretcher right_auto_init() {
        rightexpand.setPosition(rightback);
        return this;
    }

    public Stretcher RightExpandBack() {
        rightexpand.setPosition(rightback);
        return this;
    }

    //单纯地控制stretcher向前伸展
    public Stretcher RightExpandForward() {
        rightexpand.setPosition(rightfront);

        return this;
    }

    // 右侧向前伸展到x厘米
    public Stretcher RightExpandTo(int pwm) {
//        rightexpand.setPosition((-0.0062*x*x*x+0.4398*x*x-18.988*x+2176.4-500.0)/2000.0);
        rightexpand.setPosition((pwm-500.0)/2000.0);
        return this;
    }
    public Stretcher left_auto_init() {
        leftexpand.setPosition(leftback);
        return this;
    }

    public Stretcher LeftExpandBack() {
        leftexpand.setPosition(leftback);
        return this;
    }

    //单纯地控制stretcher向前伸展
    public Stretcher LeftExpandForward() {
        leftexpand.setPosition(leftfront);
        return this;
    }


    public Stretcher LeftLittleForward(){
        robot.waitFor(50.0);
        leftexpand.setPosition(leftexpand.getPosition()-0.001);
        return this;
    }

    public Stretcher LeftLittleBackward(){
        robot.waitFor(50.0);
        leftexpand.setPosition(leftexpand.getPosition()+0.001);
        return this;
    }

    public Stretcher RightLittleForward(){
        robot.waitFor(50.0);
        rightexpand.setPosition(rightexpand.getPosition()+0.001);
        return this;
    }

    public Stretcher RightLittleBackward(){
        robot.waitFor(50.0);
        rightexpand.setPosition(rightexpand.getPosition()-0.001);
        return this;
    }

}