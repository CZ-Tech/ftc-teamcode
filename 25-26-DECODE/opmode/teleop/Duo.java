package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.command.Command;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.drive.OdoDrivetrain;
import org.firstinspires.ftc.teamcode.common.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.Robot;

//@TeleOp(name = "Duo", group = "Duo")
@Config
public class Duo extends LinearOpMode {
    public static double SLOW_GAIN = 0.25;
    public static int FAR_RPM1 = 3350, FAR_RPM2 = 3425, FAR_RPM3 = 3500;
    public static int NEAR_RPM1 = 2900, NEAR_RPM2 = 3000, NEAR_RPM3 = 3100;

    public static int NEAR_RPM = 3300;
    public static int FAR_PWM = 2900;
    Robot robot = new Robot();

    public boolean isAiming = false;


    @Override
    public void runOpMode() {
        robot.init(this);

//        robot.vision.init(colorLocatorGreen, colorLocatorPurple);

        telemetry.addLine("Robot ready!");

        robot.command.stopTransporting();
        robot.subsystem.classifier.out();

        if (robot.teamColor == null) robot.teamColor = Alliance.BLUE;

        if (!MixedOdo.isPoseInitialized) {
            robot.odoDrivetrain.resetYaw();
            robot.odo.reCollaborate();
        }

        telemetry.update();
//        robot.teamColor = Alliance.BLUE;
        robot.odoDrivetrain.setShootAngle(robot.teamColor.getBaseAngle());

        robot.gyroTracker.init(robot.teamColor.getBaseAngle());

//        robot.limelight.pipelineSwitch(robot.teamColor.getBaseAprilTag() % 20);
//        robot.limelight.setPollRateHz(50);
//        robot.limelight.start();

        while (!MixedOdo.isPoseInitialized && !isStarted()) {
            robot.odo.update();
            robot.telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            Gamepad preGamepad1 = new Gamepad();
            preGamepad1.copy(gamepad1);

//            double[] result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());


//            robot.telemetry.addData("Reset IMU", false);

            // update 必须每次循环最开始执行，并且整个循环内每个手柄只能执行一次。
            robot.odo.update();
            robot.gamepad1.update();
            robot.gamepad2.update();

            double distanceToGoalMM = Math.hypot(
                    (robot.teamColor.getGoalPos().getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                    (robot.teamColor.getGoalPos().getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
            );

            if (!gamepad2.ps){
                robot.gamepad2
                        .keyDown("y", () -> robot.command.shoot3Times_TELEOP(Globals.diss_near))
                ;
            } else{
                robot.gamepad2
                        .keyDown("y", () -> robot.command.shoot3Times_TELEOP(Globals.diss_far))
                ;
            }

            robot.gamepad1
                    .keyDown("right_bumper", () -> robot.gamepad1.rumble(1000))
                    .keyPress("dpad_down", () ->  {robot.subsystem.belt.down();robot.subsystem.door.spin(-1);})
                    .keyUp("dpad_down", () -> {robot.subsystem.belt.stop();robot.subsystem.door.spin(0);})
                    .keyPress("a", () ->  {robot.subsystem.belt.down();robot.subsystem.door.spin(-1);})
                    .keyUp("a", () -> {robot.subsystem.belt.stop();robot.subsystem.door.spin(0);})
            ;

//            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
////                robot.command.stopTransporting();
//                robot.subsystem.shooter.stop();
////                robot.odoDrivetrain.stopMotor();
//            }

            robot.gamepad2
                    .keyPress("a", () -> {
                        robot.subsystem.shooter.staticShoot(0);
                    })
                    .keyUp("a", () -> robot.subsystem.shooter.stop());
//                    .keyPress("b", () -> robot.subsystem.intaker.intakerIn(1))
//                    .keyUp("b", () -> robot.subsystem.intaker.intakerArmStop())
//
//                    .keyPress("x", () -> robot.subsystem.belt.up())
//                    .keyUp("x", () -> robot.subsystem.belt.stop())

            robot.gamepad1
                    .keyPress("right_bumper", () -> {
                        if (robot.gamepad1.left_bumper) robot.subsystem.door.spin(1);
                        robot.command.transportArtifact();
                    }
                    )
                    .keyUp("right_bumper", () -> {
                        robot.subsystem.door.spin(1);
                        robot.command.stopTransporting();
                    }
                    )
                    .keyPress("x", () -> robot.command.smartShootTransfer())
                    .keyUp("x", () -> robot.command.stopSmartShootTransfer())
                    .keyPress("b", () -> TaskLoopFrame.runOnce(() -> robot.command.shoot3TimesTele(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM))))
                    .keyDown("left_bumper",
                            () -> {
                        robot.subsystem.door.spin(1);
                        robot.subsystem.shooter.staticShoot(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));

                    })
                    .keyUp("left_bumper",
                            () -> {
                        robot.subsystem.shooter.stop();
                                robot.subsystem.door.spin(0);
                            }

                    )
//                    .keyDown("right_stick_button",
//                            () -> robot.command.autoSteer()
//                    )
                    .keyUp("right_bumper", () -> robot.subsystem.shooter.stop())
                    .keyPress("dpad_up",
                            () -> robot.subsystem.shooter.shoot(FAR_PWM)
                    )
                    .keyUp("dpad_up", () -> robot.subsystem.shooter.stop())
                    .keyDown("dpad_left", () -> Globals.SHOOT_DIS_OFFSET_MM += 10)
                    .keyDown("dpad_right", () -> Globals.SHOOT_DIS_OFFSET_MM -= 10)
            ;
//                    .keyUp("right_bumper", () -> robot.command.stopTransporting())

//            if (gamepad1.right_trigger > 0.5) robot.command.transportArtifact();
//            if (gamepad1.left_trigger > 0.5){
//                robot.subsystem.shooter.shoot(result[10]);
//                robot.command.autoSteer();
//            }


            robot.gamepad2
                    .keyPress("x", () -> robot.subsystem.shooter.back())
                    .keyUp("x", () -> robot.subsystem.shooter.stop())
            ;

//            if (!gamepad2.ps) {
//                robot.gamepad2
//                        .keyDown("dpad_left", () -> robot.subsystem.shooter.shoot(0, NEAR_RPM1))
//                        .keyDown("dpad_up", () -> robot.subsystem.shooter.shoot(0, NEAR_RPM2))
//                        .keyPress("dpad_right", () -> robot.subsystem.shooter.shoot(0, NEAR_RPM3))
//                        .keyUp("dpad_left", () -> robot.subsystem.shooter.stop())
//                        .keyUp("dpad_up", () -> robot.subsystem.shooter.stop())
//                        .keyUp("dpad_right", () -> robot.subsystem.shooter.stop())
//                ;
//            }
//            else {
//                robot.gamepad2
//                        .keyDown("dpad_left", () -> robot.subsystem.shooter.shoot(0, FAR_RPM1))
//                        .keyDown("dpad_up", () -> robot.subsystem.shooter.shoot(0, FAR_RPM2))
//                        .keyPress("dpad_right", () -> robot.subsystem.shooter.shoot(0, FAR_RPM3))
//                        .keyUp("dpad_left", () -> robot.subsystem.shooter.stop())
//                        .keyUp("dpad_up", () -> robot.subsystem.shooter.stop())
//                        .keyUp("dpad_right", () -> robot.subsystem.shooter.stop())
//                ;
//            }

            ;

//                    .keyDown("dpad_left", () -> robot.command.autoSteer())

//                    .keyPress("dpad_right", () -> robot.subsystem.intaker.intakerIn(-1))
//                    .keyUp("dpad_right", () -> robot.subsystem.intaker.intakerArmStop())

            robot.gamepad1
                    .keyDown("share", "options", () -> robot.odoDrivetrain.resetYaw());

            robot.gamepad2
//                    .keyDown("right_bumper", () -> robot.subsystem.classifier.in())
                    .keyPress("left_bumper", () -> robot.command.leaveArtifact())
                    .keyUp("left_bumper", () -> robot.command.stopTransporting())

                    .keyPress("right_bumper", () -> robot.subsystem.belt.down())
                    .keyUp("right_bumper", () -> robot.subsystem.belt.stop())
            ;
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


            robot.gamepad1
                    .keyToggle("y", () -> isAiming = true, () -> isAiming = false);
//            if (isAiming || this.gamepad1.left_stick_button) robot.autoAimController.aimAtTarget(robot, robot.teamColor.getGoalPos(),
//                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * -gamepad1.left_stick_y : -gamepad1.left_stick_y,
//                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * gamepad1.left_stick_x : gamepad1.left_stick_x);
//            else robot.odoDrivetrain.driveRobotFieldCentric(
//                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * -gamepad1.left_stick_y : -gamepad1.left_stick_y,
//                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * gamepad1.left_stick_x : gamepad1.left_stick_x,
//                    gamepad1.right_trigger > 0.5 ? SLOW_GAIN * gamepad1.right_stick_x : gamepad1.right_stick_x
//            );

            if (isAiming || this.gamepad1.left_stick_button) robot.autoAimController.aimAtTarget(robot, robot.teamColor.getGoalPos(),
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x);
            else robot.odoDrivetrain.driveRobotFieldCentricWithTeleOpHeadReset(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
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



//            if (robot.visionC270.getColor() == 1) robot.telemetry.addData("Artifact", "Purple");
//            else if (robot.visionC270.getColor() == -1) robot.telemetry.addData("Artifact", "Green");
//            else robot.telemetry.addData("Artifact", "None");

            robot.telemetry.addData("X", robot.odo.getPosition().getX(DistanceUnit.INCH));
            robot.telemetry.addData("Y", robot.odo.getPosition().getY(DistanceUnit.INCH));

            robot.telemetry.addData("左发射速度",robot.subsystem.shooter.left.getVelocity()/28*60);
            robot.telemetry.addData("右发射速度",robot.subsystem.shooter.right.getVelocity()/28*60);
//            robot.telemetry.addData("tx", result[6]);
//            robot.telemetry.addData("ty", result[7]);
//            robot.telemetry.addData("dis", result[10]);
//            robot.telemetry.addData("Unnormalized Heading", robot.odoDrivetrain.getHeading(UnnormalizedAngleUnit.DEGREES));
            robot.telemetry.addData("Normalized Heading", robot.odoDrivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("Original Heading", robot.odo.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("CurrDistance", distanceToGoalMM);
            robot.telemetry.addData("目标转速",Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
//            robot.telemetry.addData("apriltag_id", robot.visionLimelight.getTargetAprilTag());

//            robot.telemetry.addData("Offset", robot.gyroTracker.getCurrentOffset(UnnormalizedAngleUnit.DEGREES));
//            robot.telemetry.addData("Pipeline", robot.limelight.getStatus().getPipelineIndex());
//            robot.telemetry.addData("FPS", robot.limelight.getStatus().getFps());
//            robot.telemetry.addData("Connection Info", robot.limelight.getConnectionInfo());
//            robot.telemetry.addData("LimelightRunning", robot.limelight.isRunning());
//            robot.telemetry.addData("LimelightConnecting", robot.limelight.isConnected());

//            robot.llPos.getPos();

            robot.telemetry.addData("OdoInited", MixedOdo.isPoseInitialized);


            robot.telemetry.update();
        }
        TaskLoopFrame.stopAndClearAll();
    }

//    @Disabled
    @TeleOp(name = "Duo🔴", group = "Duo")
    public static class DuoRed extends Duo {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            OdoDrivetrain.angleOffset = -90;
            super.runOpMode();
        }
    }

//    @Disabled
    @TeleOp(name = "Duo🔵", group = "Duo")
    public static class DuoBlue extends Duo {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            OdoDrivetrain.angleOffset = 90;
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

