package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.command.AutoAimController;
import org.firstinspires.ftc.teamcode.common.command.Command;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "ShootSpeedTest", group = "Test")
@Config
public class ShootSpeedTest extends LinearOpMode {
    public static double SLOW_GAIN = 0.25;
    public static int FAR_PWM = 2900;
    public static int curr_rpm = 2900;
    Robot robot = new Robot();

    AutoAimController autoAimController = new AutoAimController();


    @Override
    public void runOpMode() {
        robot.init(this);

//        robot.vision.init(colorLocatorGreen, colorLocatorPurple);

        telemetry.addLine("Robot ready!");

        robot.command.stopTransporting();
        robot.subsystem.classifier.out();
        robot.odoDrivetrain.resetYaw();
        telemetry.update();

        robot.odo.reCollaborate();

        while (!MixedOdo.isPoseInitialized && !isStarted()) {
            robot.odo.update();
            robot.telemetry.update();
        };

        waitForStart();
        while (opModeIsActive()) {
            Gamepad preGamepad1 = new Gamepad();
            preGamepad1.copy(gamepad1);

            // update 必须每次循环最开始执行，并且整个循环内每个手柄只能执行一次。
            robot.odo.update();
            robot.gamepad1.update();
            robot.gamepad2.update();

            double distanceToGoalMM = Math.hypot(
                    (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                    (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
            ) + 228;


            robot.gamepad1
                    .keyPress("right_bumper",
                            () -> {
                                if (robot.gamepad1.left_bumper) {
                                    robot.command.smartShootTransfer();
                                }
                                else robot.command.transportArtifact();

                            }
                    )
                    .keyUp("right_bumper",
                            () -> {
                                robot.subsystem.door.spin(0);
                                robot.command.stopTransporting();
                                robot.command.stopSmartShootTransfer();
                            }
                    )
                    .keyPress("x", () -> robot.command.smartShootTransfer())
                    .keyUp("x", () -> robot.command.stopSmartShootTransfer())
                    .keyDown("left_bumper",
                            () -> {
                                robot.subsystem.door.spin(1);
                                robot.subsystem.shooter.staticShoot(curr_rpm);
                            })
                    .keyUp("left_bumper",
                            () -> {
                                robot.subsystem.door.spin(0);
                                robot.subsystem.shooter.stop();
                            }
                    )
                    .keyUp("right_bumper", () -> robot.subsystem.shooter.stop())
                    .keyPress("dpad_down", () ->  {robot.subsystem.belt.down();robot.subsystem.door.spin(-1);})
                    .keyUp("dpad_down", () -> {robot.subsystem.belt.stop();robot.subsystem.door.spin(0);})
                    .keyPress("dpad_left", () -> curr_rpm ++)
                    .keyPress("dpad_right", () -> curr_rpm --)
                    .keyPress("b", () -> TaskLoopFrame.runOnce(() -> robot.command.shoot3TimesTele(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM))))
            ;


            robot.gamepad2
                    .keyPress("x", () -> robot.subsystem.shooter.back())
                    .keyUp("x", () -> robot.subsystem.shooter.stop())
            ;

            robot.gamepad1
                    .keyDown("share", "options", () -> robot.odoDrivetrain.resetYaw());


            if (gamepad1.y) autoAimController.aimAtTarget(robot, Globals.BLUE_GOAL_POS,
                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * -gamepad1.left_stick_y : -gamepad1.left_stick_y,
                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * gamepad1.left_stick_x : gamepad1.left_stick_x);
            else robot.odoDrivetrain.driveRobotFieldCentricWithTeleOpHeadReset(
                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * -gamepad1.left_stick_y : -gamepad1.left_stick_y,
                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * gamepad1.left_stick_x : gamepad1.left_stick_x,
                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * gamepad1.right_stick_x : gamepad1.right_stick_x
            );

            robot.telemetry.addData("left_stick_x", gamepad1.left_stick_x);
            robot.telemetry.addData("left_stick_y", gamepad1.left_stick_y);
            robot.telemetry.addData("right_stick_x", gamepad1.right_stick_x);
            robot.telemetry.addData("right_stick_y", gamepad1.right_stick_y);

            robot.telemetry.addData("X", robot.odo.getPosition().getX(DistanceUnit.INCH));
            robot.telemetry.addData("Y", robot.odo.getPosition().getY(DistanceUnit.INCH));

            robot.telemetry.addData("左发射速度",robot.subsystem.shooter.left.getVelocity()/28*60);
            robot.telemetry.addData("右发射速度",robot.subsystem.shooter.right.getVelocity()/28*60);
            robot.telemetry.addData("Normalized Heading", robot.odoDrivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("Original Heading", robot.odo.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("OdoInited", MixedOdo.isPoseInitialized);
            robot.telemetry.addData("CurrDistance", distanceToGoalMM);
            robot.telemetry.addData("currRPM", curr_rpm);


            robot.telemetry.update();
        }
    }

}
//dis rpm
// 3999.56, 3696
// 3571,    3560
// 3007.5,  3264
// 2655,    3305
// 2392.73, 3285
// 2160.2,  3123
// 1867.1,  3036
// 1461.8,  2965
// 1298.7,  2943
// 1019,    2940
// 950,     2936


// 3999.56
// 3571
// 3007.5
// 2655
// 2392.73
// 2160.2
// 1867.1
// 1461.8
// 1298.7
// 1019
// 950