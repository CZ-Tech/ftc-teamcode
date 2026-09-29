package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;

@Disabled
@Config
@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
    public static double RPM = 3100, UP_P = 0.6;

    Robot robot = new Robot();
    @Override
    public void runOpMode(){

        robot.teamColor = Alliance.BLUE;

        robot.init(this);

        robot.gyroTracker.init(robot.teamColor.getBaseAngle());

        robot.limelight.pipelineSwitch(robot.teamColor.getBaseAprilTag() % 20);
        robot.limelight.setPollRateHz(50);
        robot.limelight.start();


        waitForStart();

        while(opModeIsActive()) {

            robot.odo.update();
//            robot.odoDrivetrain.setDrivePower(1, 0, 0, 0);
//            robot.waitFor(2000);
//            robot.odoDrivetrain.setDrivePower(0, 1, 0, 0);
//            robot.waitFor(2000);
//            robot.odoDrivetrain.setDrivePower(0, 0, 1, 0);
//            robot.waitFor(2000);
//            robot.odoDrivetrain.setDrivePower(0, 0, 0, 1);
//            robot.waitFor(2000);
//
//            robot.odoDrivetrain.setDrivePower(1, 1, 0, 0);
//            robot.waitFor(2000);
//            robot.odoDrivetrain.setDrivePower(0, 0, 1, 1);
//            robot.waitFor(2000);
//
//            robot.odoDrivetrain.setDrivePower(1, 1, 1, 1);
//            robot.waitFor(2000);
//
//            robot.odoDrivetrain.driveRobot(1, 0, 0);
//            robot.waitFor(2000);

            robot.gamepad1.update();

            double[] result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());

            robot.gamepad1
                    .keyPress("a", () -> robot.subsystem.shooter.staticShoot( RPM))
                    .keyUp("a", () -> robot.subsystem.shooter.stop())

                    .keyPress("y", () -> robot.subsystem.belt.up(UP_P))
                    .keyUp("y", () -> robot.subsystem.belt.stop())

                    .keyPress("dpad_up", () -> robot.subsystem.belt.down())
                    .keyUp("dpad_up", () -> robot.subsystem.belt.stop())

                    .keyPress("dpad_down", () -> robot.subsystem.intaker.intakerIn(1))
                    .keyUp("dpad_down", () -> robot.subsystem.intaker.intakerArmStop())

                    .keyDown("share", "options", () -> robot.odoDrivetrain.resetYaw())

                    .keyPress("ps", () -> robot.subsystem.shooter.shoot(result[10]))
                    .keyUp("ps", () -> robot.subsystem.shooter.stop())

                    .keyDown("dpad_left", () -> robot.command.autoSteer())

                    .keyDown("right_bumper", () -> robot.command.pushWheel())
                    .keyDown("left_bumper", () -> robot.command.scrollBack())
            ;

            robot.odoDrivetrain.driveRobotFieldCentricWithTeleOpHeadReset(
                    gamepad1.right_trigger > 0.5 ? 0.4 * -gamepad1.left_stick_y : -gamepad1.left_stick_y,
                    gamepad1.right_trigger > 0.5 ? 0.4 * gamepad1.left_stick_x : gamepad1.left_stick_x,
                    gamepad1.right_trigger > 0.5 ? 0.4 * gamepad1.right_stick_x : gamepad1.right_stick_x
            );



//            robot.telemetry.addData("Offset", robot.gyroTracker.getCurrentOffset(UnnormalizedAngleUnit.DEGREES));
//            robot.telemetry.addData("Unnormalized Heading", robot.odoDrivetrain.getHeading(UnnormalizedAngleUnit.DEGREES));
            robot.telemetry.addData("Normalized Heading", robot.odoDrivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("左发射速度",robot.subsystem.shooter.left.getVelocity()/28*60);
            robot.telemetry.addData("右发射速度",robot.subsystem.shooter.right.getVelocity()/28*60);
            robot.telemetry.addData("ty", result[7]);
            robot.telemetry.addData("dis", result[10]);
            robot.telemetry.addData("tx", result[6]);
            robot.telemetry.addData("Pipeline", robot.limelight.getStatus().getPipelineIndex());
            robot.telemetry.addData("FPS", robot.limelight.getStatus().getFps());
            robot.telemetry.addData("Connection Info", robot.limelight.getConnectionInfo());
            robot.telemetry.addData("LimelightRunning", robot.limelight.isRunning());
            robot.telemetry.addData("LimelightConnecting", robot.limelight.isConnected());
            robot.telemetry.update();
        }

//        robot.limelight.stop();
    }
}

/*
1.403 3100
1.24 3000
1.50 3180
2.07 3500
2.075 3570
 */