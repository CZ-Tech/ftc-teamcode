package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "测试风骚移动")
public class Move extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        double v = 0.3;
        double d1 = -6;
        double d2 = 12;
        int a = 0;
        int mul = 1;
        double n = Math.sqrt(d1 * d1 + d2 * d2);
        int lfp, rfp, rbp, lbp, maxp;
        DcMotorEx busyMotor;
        robot.init();
        waitForStart();
        robot.resetYaw();
        robot.initAprilTagVision();
        robot.setManualExposure(6, 250);

        waitForStart();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            telemetry.addData("mul-按住options键10倍修改下方值，", mul);
            telemetry.addData("v-按左右摇杆按钮修改", "%.2f", v);
            telemetry.addData("d1/y/前轮功率-按十字键上下修改", d1);
            telemetry.addData("d2/x/后轮功率-按十字键左右修改", d2);
            telemetry.addData("a-按左右肩键修改", a);
            telemetry.addData("heading", robot.getHeading());
            telemetry.addData("XBOX", "A-放线;B-转圈;X-斜走;Y-测试AprilTag");
            telemetry.addData("PS  ", "X-放线;O-转圈;□-斜走;△-测试AprilTag");

            if (currentGamepad1.share && currentGamepad1.options) robot.resetYaw();
            if (currentGamepad1.options && !previousGamepad1.options) mul = 10;
            if (!currentGamepad1.options && previousGamepad1.options) mul = 1;

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) d1 = d1 + mul;
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) d1 = d1 - mul;
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) d2 = d2 + mul;
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) d2 = d2 - mul;
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) a = a + mul;
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) a = a - mul;
            if (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button)
                v = v + 0.1 * mul;
            if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button)
                v = v - 0.1 * mul;


            if (currentGamepad1.a && !previousGamepad1.a) {
                robot.resetYaw();
                robot.driveStraight(d1, 0, v);
                robot.driveStraight(d2, a, v);
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                while (opModeIsActive() && robot.getHeading() > -135)
                    robot.setDrivePower(-d1 / 100, d1 / 100, d2 / 100, -d2 / 100);
                robot.stopMotor();
                telemetry.addData("角度", robot.getHeading());
                telemetry.update();
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
//                robot.resetYaw();
                robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.to(d1,d2,a);
//                robot.driveStrafe(0.5,24,0);

                lfp = (int) ((d1 + d2) * RobotHardware.COUNTS_PER_INCH);
                rfp = (int) ((d1 - d2) * RobotHardware.COUNTS_PER_INCH);
                rbp = (int) ((d1 + d2) * RobotHardware.COUNTS_PER_INCH);
                lbp = (int) ((d1 - d2) * RobotHardware.COUNTS_PER_INCH);
                robot.leftFrontDrive.setTargetPosition(lfp);
                robot.rightFrontDrive.setTargetPosition(rfp);
                robot.leftBackDrive.setTargetPosition(lbp);
                robot.rightBackDrive.setTargetPosition(rbp);

                robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.driveRobotFieldCentric(v * d1 / n, v * d2 / n, a / 100.0);
                lfp = Math.abs(lfp);
                rfp = Math.abs(rfp);
                lbp = Math.abs(lbp);
                rbp = Math.abs(rbp);

                maxp = Math.max(lfp, Math.max(rfp, Math.max(rbp, lbp)));

                busyMotor = robot.leftFrontDrive;
                if (maxp == rfp) busyMotor = robot.rightFrontDrive;
                if (maxp == lbp) busyMotor = robot.leftBackDrive;
                if (maxp == rbp) busyMotor = robot.rightBackDrive;

                while (opModeIsActive() && busyMotor.isBusy()) {
                    telemetry.addData("左前，右前，左后，右后", "%n  %d%n  %d%n  %d%n  %d",
                            robot.leftFrontDrive.getCurrentPosition(),
                            robot.rightFrontDrive.getCurrentPosition(),
                            robot.leftBackDrive.getCurrentPosition(),
                            robot.rightBackDrive.getCurrentPosition()
                    );
                    telemetry.update();
                }
                robot.stopMotor();
                robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                robot.resetYaw();
//        robot.setIntakeArmPosition(-1);
                robot
                //向前直走
                .w(26.0, 0.8)//TODO:待调试
                //启动飞轮放像素
                .setLeftIntakePower(-0.6)
                .s(2)
                .setLeftIntakePower(0)
                .sleep(200)
                ;
               robot
                //向后直走回到初始位置
                .s(4.0)
                //向右平移至后台
                .G(90)
        ;

        robot.initAprilTagVision();
        robot.setManualExposure(4, 250);
        robot.syncRun(
            () -> {
                robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);
                robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
            }

        );
//        thread.start();

        robot.turnToHeading(90,0.3);

                AprilTagDetection desiredTag = null;
                ;
                while (!robot.myOpMode.isStopRequested() && desiredTag == null)
                    desiredTag = robot.getAprilTag(1);
//
                while (!gamepad1.ps) {
                    telemetry.addData("x", desiredTag.ftcPose.x);
                    telemetry.addData("y", desiredTag.ftcPose.y);
                    telemetry.update();
                }
//                robot.driveStrafe(desiredTag.ftcPose.x * 1.02+d1, -90, v);
//                robot.driveStraight(desiredTag.ftcPose.y * 0.835-d2, -90, v);
////                robot.driveToAprilTag(7,15, -4.875, 0,v);
                robot.driveToAprilTag(8, d2, d1, 90);

            }
            if (!currentGamepad1.y && previousGamepad1.y) {
                robot.stopMotor();
            }
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                robot.rush(70, 0);
            }
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                robot.driveStraight(70, 0, 1);
            }
            telemetry.addData("左前，右前，左后，右后", "%n  %d%n  %d%n  %d%n  %d",
                    robot.leftFrontDrive.getCurrentPosition(),
                    robot.rightFrontDrive.getCurrentPosition(),
                    robot.leftBackDrive.getCurrentPosition(),
                    robot.rightBackDrive.getCurrentPosition()
            );
            telemetry.update();
        }
    }
}
