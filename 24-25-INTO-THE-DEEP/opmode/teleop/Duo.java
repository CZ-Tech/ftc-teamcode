package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.GamepadRumble;
import org.firstinspires.ftc.teamcode.common.util.LedColor;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;
import org.firstinspires.ftc.teamcode.common.util.status.GrabberStat;

//@Confg
@Disabled
@TeleOp(name = "Duo", group = "Tele")
public class Duo extends LinearOpMode {
    Robot robot = new Robot();
    public boolean StretcherStat = false;
    public static double TURN_GAIN = 0.3;
    public static double SPEED_GAIN = 0.4;
    boolean DetectingGamepadStat = true;

    public boolean ArmState() {
        return robot.subsystem.arm.leftArmMotor.getCurrentPosition() > 400 || robot.subsystem.arm.rightArmMotor.getCurrentPosition() < -400;
    }

    public void SetGamepad(boolean gamepadStat){
        DetectingGamepadStat = gamepadStat;
    }

    public void runOpMode() {
        robot.init(this);
        robot.opModeState = OpModeState.Duo;
        robot.telemetry.addData("Status", "Ready");
        robot.command.IngrabberInit();
        robot.subsystem.grabber.grab();

        robot.subsystem.grabber.stat = GrabberStat.GRAB;

        gamepad1.runRumbleEffect(GamepadRumble.fullPowerRumble);
        gamepad2.runRumbleEffect(GamepadRumble.fullPowerRumble);

        gamepad1.runLedEffect(LedColor.RGBEffect);
        gamepad2.runLedEffect(LedColor.RGBEffect);

        robot.limelight.stop();

        waitForStart();

        robot.runtime.reset();

        gamepad1.setLedColor(235, 115, 255, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(115, 255, 235, Gamepad.LED_DURATION_CONTINUOUS);

        robot.odoDrivetrain.resetYaw();
        while (opModeIsActive()) {
            Gamepad preGamepad2 = new Gamepad();
            preGamepad2.copy(gamepad2);

            robot.gamepad1.update();
            robot.gamepad2.update();

            robot.odo.update();
            if (gamepad1.share && gamepad1.options) {
                robot.odoDrivetrain.resetYaw();
                telemetry.addData("resetIMU", 1);
            } else {
                telemetry.addData("resetIMU", 0);
            }

            if(gamepad2.options && !preGamepad2.options){
                SetGamepad(!DetectingGamepadStat);
            }

            robot.syncRun(
                    () -> {
                        while (gamepad2.right_stick_y > 0.5){
                            robot.subsystem.stretcher.RightLittleForward();
                        }
                    },
                    () -> {
                        while (gamepad2.right_stick_y < -0.5){
                            robot.subsystem.stretcher.RightLittleBackward();
                        }
                    },
                    () -> {
                        while (gamepad2.right_stick_y > 0.5 && gamepad2.right_trigger> 0.5){
                            robot.subsystem.stretcher.LeftLittleForward();
                        }
                    },
                    () -> {
                        while (gamepad2.right_stick_y < -0.5 && gamepad2.right_trigger > 0.5){
                            robot.subsystem.stretcher.LeftLittleBackward();
                        }
                    }
            );

//            if (DetectingGamepadStat){
//            robot.odoDrivetrain.driveRobotFieldCentric(
//                    (gamepad1.left_bumper || StretcherStat || ArmState())
//                            ? SPEED_GAIN * -sss(gamepad1.left_stick_y)
//                            : -sss(gamepad1.left_stick_y),
//                    (gamepad1.left_bumper || StretcherStat || ArmState())
//                            ? SPEED_GAIN * sss(gamepad1.left_stick_x)
//            : sss(gamepad1.left_stick_x),
//                    (gamepad1.left_bumper || StretcherStat || ArmState())
//                            ? TURN_GAIN * sss(1 * gamepad1.right_stick_x * (1 + 0.5 * (gamepad1.left_stick_y * gamepad1.left_stick_y) + 0.5 * (gamepad1.left_stick_x * gamepad1.left_stick_x)))
//                            : sss(1 * gamepad1.right_stick_x * (1 + 0.5 * (gamepad1.left_stick_y * gamepad1.left_stick_y) + 0.5 * (gamepad1.left_stick_x * gamepad1.left_stick_x))
//
//                    )
//            );
//            } else {
            robot.odoDrivetrain.driveRobotFieldCentric(
                    (gamepad1.right_trigger > 0.3)
                            ? SPEED_GAIN * -reCulcGamepad(gamepad1.left_stick_y)
                            : -reCulcGamepad(gamepad1.left_stick_y),
                    (gamepad1.right_trigger > 0.3)
                            ? SPEED_GAIN * reCulcGamepad(gamepad1.left_stick_x)
                            : reCulcGamepad(gamepad1.left_stick_x),
                    (gamepad1.right_trigger > 0.3)
                            ? TURN_GAIN * reCulcGamepad(1 * gamepad1.right_stick_x * (1 + 0.5 * (gamepad1.left_stick_y * gamepad1.left_stick_y) + 0.5 * (gamepad1.left_stick_x * gamepad1.left_stick_x)))
                            : reCulcGamepad(1 * gamepad1.right_stick_x * (1 + 0.5 * (gamepad1.left_stick_y * gamepad1.left_stick_y) + 0.5 * (gamepad1.left_stick_x * gamepad1.left_stick_x))
                    )
            );
//            }


            robot.gamepad1
                    .keyDown("dpad_left", () -> robot.limelight.setTargetColor(Alliance.RED))
                    .keyDown("dpad_up", () -> robot.limelight.setTargetColor(Alliance.YELLOW))
                    .keyDown("dpad_right", () -> robot.limelight.setTargetColor(Alliance.BLUE))
                    .keyPress("dpad_down", () -> robot.subsystem.arm.oneShot(-1))
                    .keyUp("dpad_down", () -> robot.subsystem.arm.oneShotStop())
                    ;

            robot.gamepad1
                    .keyPress("right_bumper", () -> gamepad2.rumble(0, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS))
                    .keyPress("left_bumper", () -> gamepad2.rumble(1, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS))
                    .keyPress("ps", () -> gamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS))
                    .keyUp("right_bumper", () -> gamepad2.stopRumble())
                    .keyUp("left_bumper", () -> gamepad2.stopRumble())
                    .keyUp("ps", () -> gamepad2.stopRumble())

                    .keyDown("b", () -> {
                        robot.command.SlamFromTop();
                    })

                    .keyDown("x", () -> {
                        switch (robot.subsystem.grabber.stat){
                            case GRAB: robot.subsystem.grabber.release();break;
                            case RELEASE: robot.subsystem.grabber.grab();break;
                            default: break;
                        }
                    })//按下a松开/闭合爪子

                    .keyDown("a", () -> robot.command.grabup2())
                    .keyPress("y", () -> robot.subsystem.arm.oneShot())
                    .keyUp("y", () -> robot.subsystem.arm.oneShotStop())
            ;

            robot.gamepad2
                    .keyPress("left_bumper", () -> {
                        robot.subsystem.arm.up(1);
                    })//按住left_bumper让机械臂向上升
                    .keyUp("left_bumper", () -> {
                        robot.subsystem.arm.stop();
                    })//松开停止
                    .keyPress("right_bumper", () -> {
                        robot.subsystem.arm.down(0.5);
                    })//按住right_bumper让机械臂向上升
                    .keyUp("right_bumper", () -> {
                        robot.subsystem.arm.stop();
                    })//松开停止

//                    .keyToggle("y", () -> robot.subsystem.thrower.Lift(),
//                            () -> robot.subsystem.thrower.Down())

                    .keyDown("b", () -> {
                        robot.command.SlamFromTop();
                    })

                    .keyDown("x", () -> {
                        switch (robot.subsystem.grabber.stat){
                            case GRAB: robot.subsystem.grabber.release();break;
                            case RELEASE: robot.subsystem.grabber.grab();break;
                            default: break;
                        }
                    })//按下a松开/闭合爪子

                    .keyDown("a", () -> robot.command.grabup2())
            ;


            if (!(gamepad2.right_trigger > 0.5)) {
                robot.gamepad2
                        .keyDown("ps", () -> {
                            gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                                    .addStep(1, 1, 100)
                                    .addStep(0, 0, 100)
                                    .addStep(1, 1, 100)
                                    .addStep(0, 0, 100)
                                    .addStep(1, 1, 100)
                                    .build()
                            );
                            robot.command.turnForSample();
                        })

                        .keyDown("dpad_left", () -> {
                            robot.syncRun(() -> {
                                robot.subsystem.claw.down();

                                robot.waitFor(400).subsystem.claw.grab();

                                robot.waitFor(600);
                                robot.command.right_IngrabberPut();
                            });
                        })

                        .keyDown("dpad_right", () -> {
                            robot.syncRun(() -> {
                                robot.subsystem.stretcher.RightExpandForward();

                                robot.waitFor(300);
                                robot.subsystem.claw.release();

                                robot.waitFor(200);
                                robot.command.right_IngrabberPut();
                            });
                        })

                        .keyDown("dpad_down", () -> {
                            robot.command.right_IngrabberPut();
                            StretcherStat = false;
                        })
                        .keyDown("dpad_up", () -> {
                            robot.command.right_IngrabberEatForOpMode();
                            StretcherStat = true;
                        })
                ;

            } else {
                robot.gamepad2
                        .keyDown("dpad_down", () -> {
                            robot.command.left_IngrabberPut();
                            StretcherStat = false;
                        })//让左stretcher向前伸，准备好吃sample

                        .keyDown("dpad_up", () -> {
                            robot.command.left_IngrabberEatForOpMode();
                            StretcherStat = true;
                        })//让左stretcher向后缩，并吐出sample


                        .keyPress("dpad_left", () -> robot.subsystem.ingrabber.left_takeIn())//按住dpad_left让右ingrabber吃样本
                        .keyUp("dpad_left", () -> robot.subsystem.ingrabber.left_stop())//松开停止转动
                        .keyPress("dpad_right", () -> robot.subsystem.ingrabber.left_takeOut())
                        .keyUp("dpad_right", () -> robot.subsystem.ingrabber.left_stop())
                ;
            }

            robot.limelight.update();

            robot.telemetry.addData("Status", "Running");
            robot.telemetry.addData("Run time", robot.runtime.seconds());
            robot.telemetry.addData("Right Trigger", gamepad2.right_trigger);
            robot.telemetry.addData("Slow Mode", gamepad1.left_bumper);
            robot.telemetry.addData("Voltage", robot.getVoltage());

            robot.telemetry.addData("Heading", robot.odoDrivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("X Position", robot.pinpointTrajectory.getX());
            robot.telemetry.addData("Y Position", robot.pinpointTrajectory.getY());

            robot.telemetry.addData("LeftMotorMoveCounts", robot.subsystem.arm.leftArmMotor.getCurrentPosition());
            robot.telemetry.addData("RightMotorMoveCounts", robot.subsystem.arm.rightArmMotor.getCurrentPosition());
            robot.telemetry.addData("LiftmotorMoveCounts", robot.subsystem.thrower.Liftmotor.getCurrentPosition());

//            robot.telemetry.addData("Camera Stat", robot.limelight.getStat() ? "Found" : "Not Found");
//            robot.telemetry.addData("X", robot.limelight.getCx());
//            robot.telemetry.addData("Y", robot.limelight.getCy());
//            robot.telemetry.addData("Cx", robot.limelight.getCx() - (int)robot.limelight.getWidth() / 2);
//            robot.telemetry.addData("Cy", robot.limelight.getCy() - (int)robot.limelight.getHeight() / 2);
//            robot.telemetry.addData("Angle", robot.limelight.getAngle());

            robot.telemetry.update();
        }
    }

    @Disabled
    @TeleOp(name = "Duo🔴", group = "Duo")
    public static class DuoRed extends Duo {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    @Disabled
    @TeleOp(name = "Duo🔵", group = "Duo")
    public static class DuoBlue extends Duo {
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