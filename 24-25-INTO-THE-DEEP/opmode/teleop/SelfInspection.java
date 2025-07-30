package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

@Disabled
@TeleOp(name = "机器自检", group = "Test")
public class SelfInspection extends LinearOpMode {
    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double lf = 0, rf = 0, lb = 0, rb = 0, arm = 0;
        robot.init(this);

        telemetry.addLine("电机端口号")
            .addData("前","%d %d",robot.drivetrain.driveLeftFront.getPortNumber(),robot.drivetrain.driveRightFront.getPortNumber())
            .addData("后","%d %d",robot.drivetrain.driveLeftBack.getPortNumber(),robot.drivetrain.driveRightBack.getPortNumber());
        telemetry.update();

        waitForStart();
//        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.drivetrain.setTargetPosition(5000);
        runtime.reset();
//        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.drivetrain.setDrivePower(0.3);
        while (opModeIsActive()) {
            robot.gamepad1.update()
                    .keyDown("y", () -> {
                        robot.odoDrivetrain.driveLeftBack.setPower(0.5);
                    })
            ;
        }
        telemetry.addLine("驱动电机自检完成,按A进行摇臂自检");
        telemetry.update();
    }
}
