package org.firstinspires.ftc.teamcode.SelfInspection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "机器自检", group = "Test")
public class SelfInspection extends LinearOpMode{
    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double lf = 0, rf = 0, lb = 0, rb = 0, arm = 0;
        robot.init();
        waitForStart();
        robot.syncRun(
                () -> {
                    while (opModeIsActive()) {
                        robot.updateButtonState();
                    }
                }
        );
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setTargetPosition(5000);
        runtime.reset();
        //robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDrivePower(1, 1, 1, 1);
        while (opModeIsActive() && !robot.a_once()) {
            if (!robot.leftFrontDrive.isBusy() && lf == 0)
                lf = runtime.seconds();
            if (!robot.rightFrontDrive.isBusy() && rf == 0)
                rf = runtime.seconds();
            if (!robot.leftBackDrive.isBusy() && lb == 0)
                lb = runtime.seconds();
            if (!robot.rightBackDrive.isBusy() && rb == 0)
                rb = runtime.seconds();
            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("左前，右前，左后，右后运行时间", "%n  %f%n  %f%n  %f%n  %f", lf, rf, lb, rb);
            telemetry.addData("左前，右前，左后，右后实际位置", "%n  %d%n  %d%n  %d%n  %d",
                    robot.leftFrontDrive.getCurrentPosition(),
                    robot.rightFrontDrive.getCurrentPosition(),
                    robot.leftBackDrive.getCurrentPosition(),
                    robot.rightBackDrive.getCurrentPosition()
            );
            telemetry.update();
        }
        telemetry.addData("驱动电机自检完成,按A进行摇臂自检", null);
        telemetry.update();
        if (robot.a_once()) {
                
        }




    }
}
