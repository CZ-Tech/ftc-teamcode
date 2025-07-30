package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Mini Power Test")
@Disabled
public class MiniTest extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode(){
        robot.init(this);

        waitForStart();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        double yPower = 0;
        double xPower = 0;
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                yPower= Range.clip(yPower+0.01,-1,1);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                yPower= Range.clip(yPower-0.01,-1,1);
            }
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                xPower= Range.clip(xPower+0.01,-1,1);
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                xPower= Range.clip(xPower-0.01,-1,1);
            }
            robot.odoDrivetrain.driveRobot(yPower,xPower,0);
            telemetry.addData("yPower","%2f 十字键上下修改此值",yPower);
            telemetry.addData("xPower","%2f 十字键左右修改此值",xPower);
            telemetry.update();
        }
    }
}
