package org.firstinspires.ftc.teamcode.common.subsystem;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;

//@Config
public class SlamDunker {

    private final Robot robot;

    public DcMotorEx slamdunkermotor;
    public ServoImplEx slamdunkerservo;
    public SlamStat stat;

    public static double GRAB = 1700;//TODO待测试
    public static double RELEASE = 1100;//TODO待测试
    public static int GROUND_POS = -500;
    public static int AIR_POS = -200;
    public static int CHAMBER_POS = -80;

    public enum SlamStat {
        ON_AIR,
        ON_GROUND,
        ON_CHAMBER
    }



    public SlamDunker(Robot robot) {
        this.robot = robot;
        slamdunkermotor = robot.hardwareMap.get(DcMotorEx.class, "slamdunker");
        slamdunkerservo = robot.hardwareMap.get(ServoImplEx.class, "slamdunkergrabber");
        slamdunkerservo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        slamdunkermotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stat = SlamStat.ON_CHAMBER;
    }

    public SlamDunker grab() {
        slamdunkerservo.setPosition((GRAB - 500.0) / 2000.0);
        return this;
    }

    public SlamDunker release() {
        slamdunkerservo.setPosition((RELEASE - 500.0) / 2000.0);
        return this;
    }

//    public boolean is_grab() {
//        return slamdunkerservo.getPosition() == GRAB;
//    }

    public SlamDunker onChamber(double power) {
        int movecounts = CHAMBER_POS;
        slamdunkermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (robot.opModeState != OpModeState.Auto) {
            if (stat == SlamStat.ON_GROUND) {
                stat = SlamStat.ON_CHAMBER;
                movecounts = slamdunkermotor.getCurrentPosition() + 500 + CHAMBER_POS;
            } else
                return this;
        }
        runMotor(movecounts, power); //TODO:调试movecounts
        return this;
    }

    public SlamDunker onChamber() {
        onChamber(0.7);
        return this;
    }

    public SlamDunker onGround() {//-500
        onGround(DcMotor.ZeroPowerBehavior.FLOAT);
        return this;
    }

    public SlamDunker onGround(DcMotor.ZeroPowerBehavior behavior) {//-500
        int movecounts = GROUND_POS;
        if (robot.opModeState != OpModeState.Auto) {
            if (stat == SlamStat.ON_CHAMBER) {
                stat = SlamStat.ON_GROUND;
                movecounts = slamdunkermotor.getCurrentPosition() - (500 + CHAMBER_POS);
            } else
                return this;
        }
        stat = SlamStat.ON_GROUND;
        slamdunkermotor.setZeroPowerBehavior(behavior);
        runMotor(movecounts, 0.35); //TODO:调试movecounts
        return this;
    }

    public SlamDunker onAir() {//-120
        stat = SlamStat.ON_AIR;
        slamdunkermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        runMotor(AIR_POS, 0.45); //TODO:调试movecounts
        return this;
    }

    public int getPostion() {
        return slamdunkermotor.getCurrentPosition();
    }

    private void pass() {
    }

    private void runMotor(int movecounts, double power) {

        slamdunkermotor.setTargetPosition(movecounts);
        slamdunkermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slamdunkermotor.setPower(power);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (robot.opMode.opModeIsActive() && slamdunkermotor.isBusy() && runtime.milliseconds() <= 3000) {
            pass();
        }
        slamdunkermotor.setPower(0);
        slamdunkermotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
