package org.firstinspires.ftc.teamcode.common.subsystem;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.status.GrabberStat;

//@Confg
public class Grabber {
    private final Robot robot;
    public ServoImplEx grabber;
    public DcMotorEx rotation;

    public GrabberStat stat = GrabberStat.GRAB;

    public static double GRAB_POS = 2000;//TODO 调试
    public static double RELEASE_POS = 1500;//TODO 调试


    public enum Pos {
        GRAB(GRAB_POS),
        RELEASE(RELEASE_POS);
//        HALF_RELEASE(HALF_RELEASE_POS);

        public final double position;

        Pos(double position) {
            this.position = position;
        }

        public double getPosition() {
            return (this.position - 500.0) / 2000.0;
        }
    }

    public Grabber(Robot robot) {
        this.robot = robot;
        grabber = robot.hardwareMap.get(ServoImplEx.class, "grabber");
        grabber.setPwmRange(new PwmControl.PwmRange(500, 2500));

    }


    public Grabber setGrabberPos(Pos pos) {
        grabber.setPosition(pos.getPosition());
        return this;
    }

    //设置爪子舵机，让其加紧
    public Grabber grab() {
        stat = GrabberStat.GRAB;
        return this.setGrabberPos(Pos.GRAB);
    }

    //设置爪子舵机，让其松开
    public Grabber release() {
        stat = GrabberStat.RELEASE;
        return this.setGrabberPos(Pos.RELEASE);
    }

    public Grabber toggle() {
        return robot.subsystem.grabber.stat == GrabberStat.GRAB ? release() : grab();
    }

    //设置爪子舵机，让其半松开
//    public Grabber half_release(){
//        return this.setGrabberPos(Pos.HALF_RELEASE);
//    }
}
