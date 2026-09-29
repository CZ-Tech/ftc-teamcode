package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;

@Disabled
@TeleOp(name = "TestOp", group = "Test")
public class TestOp extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode(){
        robot.init(this);

        robot.limelight.stop();

        waitForStart();

        while (opModeIsActive()){
            robot.odo.update();

            robot.gamepad1.update();

            robot.gamepad1
                    .keyPress("a", () -> robot.subsystem.intaker.intakerIn(1))
                    .keyUp("a", () -> robot.subsystem.intaker.intakerArmStop())

                    .keyPress("y", () -> robot.subsystem.belt.up())
                    .keyUp("y", () -> robot.subsystem.belt.stop())

                    .keyPress("dpad_down", () -> robot.subsystem.intaker.intakerIn(-1))
                    .keyUp("dpad_down", () -> robot.subsystem.intaker.intakerArmStop())

                    .keyPress("dpad_up", () -> robot.subsystem.belt.down())
                    .keyUp("dpad_up", () -> robot.subsystem.belt.stop())

                    .keyToggle("dpad_down", () -> robot.command.pushWheel(),
                            ()->robot.command.scrollBack())
            ;

            robot.odoDrivetrain.driveRobotFieldCentricWithTeleOpHeadReset(
                    gamepad1.right_trigger > 0.5 ? 0.4 * -gamepad1.left_stick_y : -gamepad1.left_stick_y,
                    gamepad1.right_trigger > 0.5 ? 0.4 * gamepad1.left_stick_x : gamepad1.left_stick_x,
                    gamepad1.right_trigger > 0.5 ? 0.4 * gamepad1.right_stick_x : gamepad1.right_stick_x
            );
        }
    }
}
