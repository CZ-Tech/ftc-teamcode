package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "测试", group = "Test")
public class Move_Test extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    public void runOpMode() {
        robot.init();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        double armPosition = 0;
        double power = 0;
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.resetYaw();
        robot.to(24, 24, 0, 0.5);
        robot.syncRun(
                () -> {
                    int i = 0;
                    while (opModeIsActive() && i < 10) {
                        robot.zzz(1000);
                        telemetry.addData("run1", i);
                        i++;
                    }
                },
                () -> {
                    int j = 0;
                    while (opModeIsActive() && j < 10) {
                        robot.zzz(500);
                        telemetry.addData("run2", j);
                        j++;
                    }
                }
        );
        waitForStart();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            armPosition = robot.intakeArm.getPosition();

//            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
//                armPosition = Range.clip(armPosition + 0.01, -1, 1);
//                robot.setIntakeArmPosition(armPosition);
//            }
//            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
//                armPosition = Range.clip(armPosition - 0.01, -1, 1);
//                robot.setIntakeArmPosition(armPosition);
//            }
//            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
//                power = Range.clip(power + 0.01, 0, 1);
//                robot.setIntakePower(power);
//            }
//            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
//                power = Range.clip(power - 0.01, 0, 1);
//                robot.setIntakePower(power);
//            }
//            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
//                xPower= Range.clip(xPower+0.01,-1,1);
//            }
//            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
//                xPower= Range.clip(xPower-0.01,-1,1);
//        }
            telemetry.addData("armPosition", "%2f 十字键上下修改此值", armPosition);
            telemetry.addData("Power","%2f 十字键左右修改此值",power);
            telemetry.update();
        }

//        robot.driveStraight(0.3,-30.0,0.0);
//        robot.driveStrafe(0.3,30.0,90.0);

    }
}
