package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MotorTest", group = "Team19656")
@Disabled
public class MotorTest extends LinearOpMode {
    DcMotor d1;

    @Override
    public void runOpMode() {
        d1 = hardwareMap.get(DcMotor.class, "m1");

        d1.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.dpad_down) {
                d1.setPower(1);
            } else if (gamepad2.dpad_up) {
                d1.setPower(-1);
            } else {
                d1.setPower(0);
            }

            telemetry.addData("Servo1 %f", d1.getPower());
            telemetry.update();
        }
    }
}
