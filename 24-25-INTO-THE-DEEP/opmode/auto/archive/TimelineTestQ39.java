package org.firstinspires.ftc.teamcode.opmode.auto.archive;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;

@Disabled
//@Config
@Autonomous(name = "AutoQ39", group = "CNSHQ3", preselectTeleOp = "Duo")
public class TimelineTestQ39 extends LinearOpMode {
    Robot robot = new Robot();

    public static double[] startPosition = {0, 0, 0, 0};

    @Override
    public void runOpMode(){
        robot.init(this);
        robot.opModeState = OpModeState.Auto;

        robot.telemetry.addData("Status", "Waiting for start");
        robot.telemetry.update();

        robot.drivetrain.resetYaw();
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odometry.resetPos();
        robot.subsystem.grabber.grab();
        waitForStart();

        robot.trajectory.startMove(0, -43, 0, 0) //重置里程计
                .addPoint(2, 0, -55, 0, 0)

                .addVelocity(-100, 100)
                .addPoint(5, -53, -50, 100, 0)
                .addPoint(7, -1, -50, 0, 0)

                .addVelocity(-75, 60)
                .addPoint(10, -53, -60, 100, 0)
                .addPoint(12, -7, -60, 0, 0)

                .addVelocity(-75, 60)
                .addPoint(15, -53, -67, 100, 0)
                .addPoint(17, -7, -67, 0, 0)

                .stopMotor()
        ;

        //wait for stop
        while (opModeIsActive());

//        robot.syncRun(()->{
//            robot.waitFor(11.5);
//
//        });
    }
}