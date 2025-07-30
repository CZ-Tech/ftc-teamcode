package org.firstinspires.ftc.teamcode.opmode.auto.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

//@Config
@Disabled
@Autonomous(name = "TestOdo2")
public class TestOdo2 extends LinearOpMode {
    Robot robot = new Robot();
    public static int distance=10;
    public static int angle = 45;
    public static double speed=0.5;
    public static double POS1 = -5;
    @Override
    public void runOpMode() {
        robot.init(this);
        robot.telemetry.addData("driveSpeed", 0);
        robot.telemetry.update();
        waitForStart();
        robot.odometry.resetPos();
        robot.syncRun(() -> {
            while (opModeIsActive()) {
                robot.odometry.update();
//                telemetry.addData("G1 lx ly rx ry", "%.2f %.2f %.2f %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
//                telemetry.addData(">", "Robot Heading = %4.0f", robot.drivetrain.getHeading());
//                telemetry.addData(">", "XPos = %4.0f  YPos = %4.0f", robot.odometry.X_Pos, robot.odometry.Y_Pos);
//                telemetry.addData(">", "△X = %4.0f  △Y = %4.0f", robot.odometry.delta_x, robot.odometry.delta_y);

                robot.telemetry.update();
            }
        });
        /*
        robot.odoDrivetrain.driveStraight(distance,  speed);
        sleep(2000);
        robot.odoDrivetrain.driveStrafe(distance, speed);
        sleep(2000);
        */
        robot.odoDrivetrain.driveStraighfe(distance, angle, speed);
//        robot.drivetrain
//                .driveStrafe(-10, 0, speed)
//        ;
    }
}
