package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;

public class TeleOpMode extends LinearOpMode {
    public final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(this);
//        robot..setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Robot ready!");
        waitForStart();
        while (opModeIsActive()) {
            robot.gamepad1
                    .update()
                    .keyPress("a", () -> telemetry.addLine("Button.A - Pressed"))
                    .keyPress("left_stick_button", () -> telemetry.addLine("Button.LEFT_STICK_BUTTON - Pressed"))
                    .keyPress("dpad_up", () -> telemetry.addLine("Button.DPAD_UP - Pressed"))
                    .keyPress("left_bumper", () -> telemetry.addLine("Button.LEFT_BUMPER - Pressed"))
                    .keyUp("a", () -> telemetry.addLine("Button.A - KeyUp"))
                    .keyDown("a", () -> telemetry.addLine("Button.A - KeyDown"))
                    .keyToggle("cross",
                            () -> System.out.println("Button.A - KeyToggle 1"),
                            () -> System.out.println("Button.A - KeyToggle 2")
                    )
            ;

            robot.drivetrain.driveRobotFieldCentric(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
            telemetry.update();
        }
    }

    @Disabled
    @TeleOp(name = "Solo🔴", group = "Solo")
    public static class SoloRed extends TeleOpMode {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            robot.opModeState = OpModeState.Solo;
            super.runOpMode();
        }
    }

    @Disabled
    @TeleOp(name = "Solo🔵", group = "Solo")
    public static class SoloBlue extends TeleOpMode {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            robot.opModeState = OpModeState.Solo;
            super.runOpMode();
        }
    }

    @Disabled
    @TeleOp(name = "Duo🔴", group = "Duo")
    public static class DuoRed extends TeleOpMode {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            robot.opModeState = OpModeState.Duo;
            super.runOpMode();
        }
    }

    @Disabled
    @TeleOp(name = "Duo🔵", group = "Duo")
    public static class DuoBlue extends TeleOpMode {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            robot.opModeState = OpModeState.Duo;
            super.runOpMode();
        }
    }
}
