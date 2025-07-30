package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.GamepadRumble;
import org.firstinspires.ftc.teamcode.common.util.LedColor;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;
import org.firstinspires.ftc.teamcode.common.util.PID;
import org.firstinspires.ftc.teamcode.common.util.status.GrabberStat;
import org.firstinspires.ftc.teamcode.common.vision.processor.ColorLocatorProcessor;
import org.firstinspires.ftc.teamcode.common.vision.processor.ColorRange;
import org.opencv.core.RotatedRect;


//@Confg
//@Disabled
@TeleOp(name = "Dabian", group = "Tele")
public class Dabian extends LinearOpMode {
    Robot robot = new Robot();
    public boolean StretcherStat = false;
    public static double TURN_GAIN = 0.5;
    public static double SPEED_GAIN = 0.5;
    private RotatedRect rect;
    private double realAngel;
    public static double fff = 0.15;
    private boolean oneShotLock = false;
    public static double Kp = 0.00342;
    public static double Ki = 0.008142;
    public static double Kd = 0.0003591;
    private PID pid = new PID(Kp, Ki, Kd, 120);

    public void runOpMode() {

        robot.init(this);
        ColorLocatorProcessor colorLocatorProcessor = new ColorLocatorProcessor(telemetry);
        robot.vision.init(colorLocatorProcessor);

        robot.opModeState = OpModeState.Duo;
        robot.telemetry.addData("Status", "Ready");

//        robot.command.IngrabberInit();
        robot.subsystem.grabber.grab();
        robot.subsystem.grabber.stat = GrabberStat.GRAB;

        robot.subsystem.newclaw.turnTo(0);
        robot.subsystem.newclaw.up();

        Globals.targetColor = ColorRange.BLUE;


        gamepad1.runRumbleEffect(GamepadRumble.fullPowerRumble);
        gamepad2.runRumbleEffect(GamepadRumble.fullPowerRumble);

        gamepad1.runLedEffect(LedColor.RGBEffect);
        gamepad2.runLedEffect(LedColor.RGBEffect);

        robot.telemetry.update();
        waitForStart();

        robot.runtime.reset();

        gamepad1.setLedColor(235, 115, 253, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(115, 255, 235, Gamepad.LED_DURATION_CONTINUOUS);

        robot.odoDrivetrain.resetYaw();
        robot.odoDrivetrain.angleOffset = Math.PI;
        while (opModeInInit() || opModeIsActive()) {

            robot.gamepad1.update();
            robot.gamepad2.update();

            robot.odo.update();


            rect = colorLocatorProcessor.getRect();
            if (rect != null) {
                realAngel = rect.size.width > rect.size.height ?
                        rect.angle - 90
                        : rect.angle;
                telemetry.addData("rect x", rect.center.x);
                telemetry.addData("rect y", rect.center.y);
                telemetry.addData("rect w", rect.size.width);
                telemetry.addData("rect h", rect.size.height);
                telemetry.addData("angle", rect.angle);
                telemetry.addData("real angle", realAngel);

            }

            if (!gamepad2.dpad_up || rect == null) {
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
            }

            robot.gamepad1
                    .keyPress("share", "options", () -> {
                        robot.odoDrivetrain.resetYaw();
                        telemetry.addData("resetIMU", 1);
                    })
//                    .keyPress("y", () -> robot.subsystem.arm.oneShot())
//                    .keyUp("y", () -> robot.subsystem.arm.oneShotStop())
//                    .keyPress("dpad_down", () -> robot.subsystem.arm.oneShot(-1))
//                    .keyUp("dpad_down", () -> robot.subsystem.arm.oneShotStop())

//                    .keyPress("right_bumper", () -> gamepad2.rumble(0, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS))
//                    .keyUp("right_bumper", () -> gamepad2.stopRumble())
//                    .keyPress("left_bumper", () -> gamepad2.rumble(1, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS))
//                    .keyUp("left_bumper", () -> gamepad2.stopRumble())
//                    .keyPress("ps", () -> gamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS))
//                    .keyUp("ps", () -> gamepad2.stopRumble())

                    .keyDown("a", robot.command::grabup2)
                    .keyDown("b", robot.command::SlamFromTop)
                    .keyDown("x", robot.subsystem.grabber::toggle)//按下a松开/闭合爪子
//                    .keyDown("ps", () -> {
//                        if (oneShotLock) return;
//                        robot.subsystem.arm.oneShotReady();
//                        oneShotLock = true;
//                    })


//                    .keyDown("dpad_left", () -> Globals.VAR1 -= 1)
//                    .keyDown("dpad_right", () -> Globals.VAR1 += 1)


                    // 自瞄吃联盟色样本
                    .keyDown("right_bumper", () -> robot.subsystem.newclaw.observer())
                    .keyPress("right_bumper", () -> aimToSample(robot.teamColor == Alliance.BLUE ? ColorRange.BLUE : ColorRange.RED))
                    .keyUp("right_bumper", this::shotToSample)

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


                    // 自瞄吃联黄色样本
//                    .keyDown("dpad_up", () -> robot.subsystem.newclaw.observer())
//                    .keyPress("dpad_up", () -> aimToSample(ColorRange.YELLOW))
//                    .keyUp("dpad_up", this::shotToSample)

                    // 投掷样本
                    .keyDown("left_bumper", () -> {
                        robot.syncRun(() -> {
                            robot.subsystem.stretcher.RightExpandForward();
                            sleep(300);
                            robot.subsystem.newclaw.drop();
                            sleep(250);
                            robot.subsystem.newclaw.observer().up();
//                            robot.subsystem.newclaw.observer();


                        });
                    })
            ;


//            robot.gamepad2
//                    .keyPress("left_bumper", () -> {
//                        robot.subsystem.arm.up(1);
//                    })//按住left_bumper让机械臂向上升
//                    .keyUp("left_bumper", () -> {
//                        robot.subsystem.arm.stop();
//                    })//松开停止
//                    .keyPress("right_bumper", () -> {
//                        robot.subsystem.arm.down(0.5);
//                    })//按住right_bumper让机械臂向上升
//                    .keyUp("right_bumper", () -> {
//                        robot.subsystem.arm.stop();
//                    })//松开停止

//                    .keyDown("a", robot.command::grabup2)
//                    .keyDown("b", robot.command::SlamFromTop)
//                    .keyDown("x", robot.subsystem.grabber::toggle)//按下a松开/闭合爪子
//                    .keyDown("y", () -> {
//                        robot.subsystem.arm.EndTop();
//                    })//按下a松开/闭合爪子
//

//            ;


            if (gamepad2.right_trigger < 0.5) {

//                robot.gamepad2
//                        .keyToggle("dpad_left", robot.subsystem.newclaw::release, robot.subsystem.newclaw::grab)
//                        .keyToggle("dpad_right", robot.subsystem.newclaw::observer, robot.subsystem.newclaw::up)
//                        .keyToggle("ps", robot.subsystem.newclaw::turnLeft, robot.subsystem.newclaw::turnRight)
//                        .keyDown("dpad_up", () -> {
//                            robot.subsystem.stretcher.RightExpandForward();
//                            robot.subsystem.newclaw.release();
//                            sleep(300);
//                            robot.subsystem.newclaw.down();
//                        })
//                        .keyDown("dpad_down", () -> robot.subsystem.newclaw.grab().observer().up());


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

//            robot.telemetry.addData("VAR1", Globals.VAR1);
            robot.telemetry.update();
        }
    }

    @Disabled
    @TeleOp(name = "Duo🔴", group = "Duo")
    public static class DuoRed extends Dabian {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            Globals.targetColor = ColorRange.RED;
            super.runOpMode();
        }
    }

    @Disabled
    @TeleOp(name = "Duo🔵", group = "Duo")
    public static class DuoBlue extends Dabian {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            Globals.targetColor = ColorRange.BLUE;
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

    private void aimToSample(ColorRange targetColor) {
        Globals.targetColor = targetColor;
//        if (rect != null) {
//            robot.odoDrivetrain.driveRobot(
//                    -gamepad1.left_stick_y * 0.5
//                            + rect.center.y < 65 ? 0.2 :
//                            rect.center.y > 225 ? -0.2
//                                    : 0
//                    ,
//                    gamepad1.left_stick_x * 0.5
//                            + Range.clip(
//                            (rect.center.x - 120) / 500 +
//                                    (rect.center.x > 120 ?
//                                            +fff
//                                            : -fff)
//                            , -0.3, 0.3)
////                    + pid.getResult(rect.center.x)
//                    ,
//                    gamepad1.right_stick_x * 0.5
//            );
//            if (
//                    rect.center.y > 60
//                            && rect.center.y < 220
//                            && rect.center.x < 130
//                            && rect.center.x > 110
//            ) {
//                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
//            } else {
//                gamepad1.stopRumble();
//            }
//        }
    }

    private void shotToSample() {
        if (rect != null) {
            robot.subsystem.newclaw.release();
            robot.odoDrivetrain.stopMotor();
            robot.subsystem.stretcher.RightExpandTo((int) (rect.center.y * rect.center.y * -0.0014 + rect.center.y * 1.5511 + 1650.6));
//            robot.subsystem.stretcher.RightExpandTo(Globals.VAR1);
            robot.subsystem.newclaw.turnTo(realAngel);
            robot.sleep(200);
            robot.subsystem.newclaw.down();
//                            robot.sleep((int)(0.005*rect.center.y*rect.center.y-2.549*rect.center.y+562.69));
            robot.sleep(330);
            robot.subsystem.newclaw.grab();
            robot.sleep(380);
            robot.subsystem.newclaw.observer();
            robot.subsystem.newclaw.up();
        }
    }
}