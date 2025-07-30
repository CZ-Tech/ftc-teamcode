package org.firstinspires.ftc.teamcode.opmode.auto.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

@Disabled
@Autonomous(name="TestOdo")
public class TestOdo extends LinearOpMode {

    Robot robot = new Robot();
    @Override
    public void runOpMode() {
        robot.init(this);
        waitForStart();
        robot.odometry.resetPos();
        robot.syncRun(() -> {
            while (opModeIsActive()) {
                robot.odometry.update();
                telemetry.addData("G1 lx ly rx ry", "%.2f %.2f %.2f %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
                telemetry.addData(">", "Robot Heading = %4.0f", robot.drivetrain.getHeading());
                telemetry.addData(">", "XPos = %4.0f  YPos = %4.0f", robot.odometry.X_Pos, robot.odometry.Y_Pos);
                telemetry.addData(">", "△X = %4.0f  △Y = %4.0f", robot.odometry.delta_x, robot.odometry.delta_y);
                telemetry.update();
            }
        });
        robot.drivetrain.holdHeading(0, 1, 1);
        robot.drivetrain.driveStraight(24, 0, 0.5);
        sleep(5000);
        robot.drivetrain.driveStrafe(24, 0, 0.5);


    }

}
