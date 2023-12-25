package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "手动阶段_Test", group = "Robot")
public class TeleOp19656_Test extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    Boolean headless= true;

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = amepad1.right_stick_x;
            if (gamepad1.options) {
                robot.resetYaw();
            }

            if(gamepad1.share)headless=!headless;
            if(headless) {
                // 基于场地中心视角的第一人称操控模式
                robot.driveRobotFieldCentric(drive, strafe, turn);
            }else{
                // 基于机器人中心视角的第一人称操控模式
                robot.driveRobot(drive, strafe, turn);
            }

            //进出像素
            leftIntaker = (gamepad2.left_bumper ? -1 : 0) + gamepad2.left_trigger;
            robot.setLeftIntakePower(-leftIntaker);
            rightIntaker = (gamepad2.right_bumper ? -1 : 0) + gamepad2.right_trigger;
            robot.setRightIntakePower(rightIntaker);

            telemetry.update();

        }
    }
}
