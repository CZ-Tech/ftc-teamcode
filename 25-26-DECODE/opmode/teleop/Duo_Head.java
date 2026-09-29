package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
@Disabled
//@TeleOp(name = "Duo", group = "Duo")
@Config
public class Duo_Head extends LinearOpMode {
    public static double SLOW_GAIN = 0.4;
    public static int FAR_RPM = 3300, NEAR_RPM = 3000;
    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(this);

//        robot.vision.init(colorLocatorGreen, colorLocatorPurple);

        telemetry.addLine("Robot ready!");

        robot.command.stopTransporting();
        robot.subsystem.classifier.out();
        robot.odoDrivetrain.resetYaw();
        telemetry.update();
//        robot.teamColor = Alliance.BLUE;
        robot.odoDrivetrain.setShootAngle(robot.teamColor.getBaseAngle());

        robot.gyroTracker.init(robot.teamColor.getBaseAngle());

        boolean steering = false;

        waitForStart();
        while (opModeIsActive()) {
            Gamepad preGamepad1 = new Gamepad();
            preGamepad1.copy(gamepad1);

            double[] result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());


//            robot.telemetry.addData("Reset IMU", false);

            // update 必须每次循环最开始执行，并且整个循环内每个手柄只能执行一次。
            robot.odo.update();
            robot.gamepad1.update();
            robot.gamepad2.update();

            if (!gamepad2.ps){
                robot.gamepad2
                        .keyDown("y", () -> robot.command.shoot3Times_TELEOP(Globals.diss_near))
                ;
            }
            else{
                robot.gamepad2
                        .keyDown("y", () -> robot.command.shoot3Times_TELEOP(Globals.diss_far))
                ;
            }

            robot.gamepad1
                    .keyDown("right_bumper", () -> robot.gamepad1.rumble(1000))
                    .keyDown("a", () -> {
                        robot.command.pushWheel();
//                        robot.waitFor(1000);
//                        robot.subsystem.door.standBy();
                    })
                    .keyPress("dpad_down", () -> robot.subsystem.belt.down())
                    .keyUp("dpad_down", () -> robot.subsystem.belt.stop())
            ;

//            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
////                robot.command.stopTransporting();
//                robot.subsystem.shooter.stop();
////                robot.odoDrivetrain.stopMotor();
//            }

            robot.gamepad2
                    .keyPress("a", () -> {
                        robot.subsystem.shooter.shoot(result[10]);
                    })
                    .keyUp("a", () -> robot.subsystem.shooter.stop());
//                    .keyPress("b", () -> robot.subsystem.intaker.intakerIn(1))
//                    .keyUp("b", () -> robot.subsystem.intaker.intakerArmStop())
//
//                    .keyPress("x", () -> robot.subsystem.belt.up())
//                    .keyUp("x", () -> robot.subsystem.belt.stop())

            robot.gamepad1
                    .keyPress("right_bumper", () -> robot.command.transportArtifact())
                    .keyUp("right_bumper", () -> robot.command.stopTransporting())
                    .keyPress("left_bumper", () -> {
                        robot.subsystem.shooter.staticShoot( FAR_RPM);
//                        robot.command.autoSteer();
                    })
                    .keyUp("left_bumper", () -> robot.subsystem.shooter.stop())

                    .keyDown("right_stick_button", () -> robot.command.autoSteer())

                    .keyUp("right_bumper", () -> robot.subsystem.shooter.stop())
                    .keyPress("dpad_up", () -> robot.subsystem.shooter.staticShoot( NEAR_RPM))
            ;
//                    .keyUp("right_bumper", () -> robot.command.stopTransporting())

//            if (gamepad1.right_trigger > 0.5) robot.command.transportArtifact();
//            if (gamepad1.left_trigger > 0.5){
//                robot.subsystem.shooter.shoot(result[10]);
//                robot.command.autoSteer();
//            }


            robot.gamepad2
                    .keyPress("x", () -> robot.subsystem.belt.down())
                    .keyUp("x", () -> robot.subsystem.belt.stop())

//                    .keyDown("y", () -> robot.command.shoot3Times(Globals.diss_near))


                    .keyDown("dpad_down", () -> robot.command.pushWheel())
                    .keyDown("dpad_up", () -> robot.command.scrollBack());

//                    .keyDown("dpad_left", () -> robot.command.autoSteer())

//                    .keyPress("dpad_right", () -> robot.subsystem.intaker.intakerIn(-1))
//                    .keyUp("dpad_right", () -> robot.subsystem.intaker.intakerArmStop())

//            robot.gamepad1
//                    .keyDown("share", "options", () -> robot.odoDrivetrain.resetYaw());

            robot.gamepad2
//                    .keyDown("right_bumper", () -> robot.subsystem.classifier.in())
                    .keyPress("left_bumper", () -> robot.command.leaveArtifact())
                    .keyUp("left_bumper", () -> robot.command.stopTransporting())

            ;
            //2
//            if (gamepad1.options && gamepad1.share && ((!preGamepad1.options && gamepad1.options) || (!preGamepad1.share && gamepad1.share))) {
////                robot.telemetry.addData("Reset IMU", true);
//                robot.odoDrivetrain.resetYaw();
//            }

//            robot.gamepad2
//                    .keyDown("a", () -> robot.command.shoot3Times(false,3500))
////                    .keyDown("a", () -> robot.command.shoot3Times(true, result[10]))
////                    .keyDown("y", () -> robot.command.pushWheel(result[10]))
////                    .keyDown("y", () -> robot.command.pushWheel(false,3500))
//
//            ;


            robot.odoDrivetrain.driveRobot(
                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * -gamepad1.left_stick_y : -gamepad1.left_stick_y,
                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * gamepad1.left_stick_x : gamepad1.left_stick_x,
                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * gamepad1.right_stick_x : gamepad1.right_stick_x
            );

//            robot.odoDrivetrain.driveRobotFieldCentric(
//                    -gamepad1.left_stick_y,
//                    gamepad1.left_stick_x,
//                    gamepad1.right_stick_x
//            );

//            robot.odoDrivetrain.driveRobotFieldCentric(
//                    reCulcGamepad(-gamepad1.left_stick_y),
//                    reCulcGamepad(gamepad1.left_stick_x),
//                    reCulcGamepad(gamepad1.right_stick_x)
//            );



            if (robot.visionC270.getColor() == 1) robot.telemetry.addData("Artifact", "Purple");
            else if (robot.visionC270.getColor() == -1) robot.telemetry.addData("Artifact", "Green");
            else robot.telemetry.addData("Artifact", "None");

            robot.telemetry.addData("X", robot.odo.getPosition().getX(DistanceUnit.INCH));
            robot.telemetry.addData("Y", robot.odo.getPosition().getY(DistanceUnit.INCH));

            robot.telemetry.addData("左发射速度",robot.subsystem.shooter.left.getVelocity()/28*60);
            robot.telemetry.addData("右发射速度",robot.subsystem.shooter.right.getVelocity()/28*60);
            robot.telemetry.addData("ty", result[7]);
            robot.telemetry.addData("dis", result[10]);
//            robot.telemetry.addData("Unnormalized Heading", robot.odoDrivetrain.getHeading(UnnormalizedAngleUnit.DEGREES));
            robot.telemetry.addData("Normalized Heading", robot.odoDrivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("apriltag_id", robot.visionLimelight.getTargetAprilTag());

//            robot.telemetry.addData("Offset", robot.gyroTracker.getCurrentOffset(UnnormalizedAngleUnit.DEGREES));
            robot.telemetry.update();
        }
        TaskLoopFrame.stopAndClearAll();
    }

    @Disabled
    @TeleOp(name = "Duo_Head🔴", group = "Duo")
    public static class DuoRed extends Duo_Head {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    @Disabled
    @TeleOp(name = "Duo_Head🔵", group = "Duo")
    public static class DuoBlue extends Duo_Head {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            super.runOpMode();
        }
    }

    private double reCulcGamepad(double v) {
        if (v > 0.0) { //若手柄存在中位漂移或抖动就改0.01
            v = 0.87 * v * v * v + 0.09;//0.09是23-24赛季底盘启动需要的功率
        } else if (v < 0.0) { //若手柄存在中位漂移或抖动就改-0.01
            v = 0.87 * v * v * v - 0.09; //三次方是摇杆曲线
        } else {
            // XBOX和罗技手柄死区较大无需设置中位附近
            // 若手柄存在中位漂移或抖动就改成 v*=13
            // 这里的13是上面的0.13/0.01=13
            v = 0;
        }
        return v;
    }
}

