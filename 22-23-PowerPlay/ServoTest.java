package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "ServoTest", group = "Team19656")
@Disabled

public class ServoTest extends LinearOpMode {
    Servo s1, s2, s3, s4, s5;
    DcMotor d1;

    @Override
    public void runOpMode() {
        d1 = hardwareMap.get(DcMotor.class, "m1");
//        s1 = hardwareMap.get(ServoImplEx.class, "s1");
//        s2 = hardwareMap.get(Servo.class, "s2");
//        s3 = hardwareMap.get(Servo.class, "s3");
//        s4 = hardwareMap.get(Servo.class, "s4");
//        s5 = hardwareMap.get(Servo.class, "s5");

        d1.setDirection(DcMotorSimple.Direction.FORWARD);
//        s1.setDirection(Servo.Direction.REVERSE);
//        s2.setDirection(Servo.Direction.REVERSE);
//        s3.setDirection(Servo.Direction.REVERSE);
//        s4.setDirection(Servo.Direction.REVERSE);
//        s5.setDirection(Servo.Direction.REVERSE);


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

//            if (gamepad2.dpad_down) {
//                s1.setPosition(1);
//            } else if (gamepad2.dpad_up) {
//                s1.setPosition(0);
//            } else {
//                s1.setPosition(0.5);
//            }

//            if (gamepad2.dpad_down) {
//                s2.setPosition(1);
//            } else if (gamepad2.dpad_up) {
//                s2.setPosition(0);
//            } else {
//                s2.setPosition(0.5);
//            }
//            if (gamepad2.dpad_down) {
//                s3.setPosition(1);
//            } else if (gamepad2.dpad_up) {
//                s3.setPosition(0);
//            } else {
//                s3.setPosition(0.5);
//            }
//            if (gamepad2.dpad_down) {
//                s4.setPosition(1);
//            } else if (gamepad2.dpad_up) {
//                s4.setPosition(0);
//            } else {
//                s4.setPosition(0.5);
//            }
//            if (gamepad2.dpad_down) {
//                s5.setPosition(1);
//            } else if (gamepad2.dpad_up) {
//                s5.setPosition(0);
//            } else {
//                s5.setPosition(0.5);
//            }
            telemetry.addData("Servo1 %f", s1);
//            telemetry.addData("Servo2 %f", s2);
//            telemetry.addData("Servo3 %f", s3);
//            telemetry.addData("Servo4 %f", s4);
//            telemetry.addData("Servo5 %f", s5);
            telemetry.update();
        }
    }
}
