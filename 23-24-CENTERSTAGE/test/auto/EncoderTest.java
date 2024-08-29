package org.firstinspires.ftc.teamcode.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "电机编码器测试", group = "Test")
public class EncoderTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double lf = 0, rf = 0, lb = 0, rb = 0;
        robot.init();
        PIDFCoefficients pidfOrigLF = robot.leftFrontDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfOrigRF = robot.rightFrontDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfOrigLB = robot.leftBackDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfOrigRB = robot.rightBackDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("P,I,D,F (LF)", "%.04f, %.04f, %.04f, %.04f",
                pidfOrigLF.p, pidfOrigLF.i, pidfOrigLF.d, pidfOrigLF.f);
        telemetry.addData("P,I,D,F (RF)", "%.04f, %.04f, %.04f, %.04f",
                pidfOrigRF.p, pidfOrigRF.i, pidfOrigRF.d, pidfOrigRF.f);
        telemetry.addData("P,I,D,F (LB)", "%.04f, %.04f, %.04f, %.04f",
                pidfOrigLB.p, pidfOrigLB.i, pidfOrigLB.d, pidfOrigLB.f);
        telemetry.addData("P,I,D,F (RB)", "%.04f, %.04f, %.04f, %.04f",
                pidfOrigRB.p, pidfOrigRB.i, pidfOrigRB.d, pidfOrigRB.f);
        telemetry.update();

        waitForStart();
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setTargetPosition(5000);
        runtime.reset();
        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDrivePower(1, 1, 1, 1);
        while (opModeIsActive()) {
            if (!robot.leftFrontDrive.isBusy() && lf == 0)
                lf = runtime.seconds();
            if (!robot.rightFrontDrive.isBusy() && rf == 0)
                rf = runtime.seconds();
            if (!robot.leftBackDrive.isBusy() && lb == 0)
                lb = runtime.seconds();
            if (!robot.rightBackDrive.isBusy() && rb == 0)
                rb = runtime.seconds();
            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("左前，右前，左后，右后", "%n %f%n  %f%n  %f%n  %f", lf, rf, lb, rb);
            telemetry.addData("左前，右前，左后，右后", "%n %d%n  %d%n  %d%n  %d",
                    robot.leftFrontDrive.getCurrentPosition(),
                    robot.rightFrontDrive.getCurrentPosition(),
                    robot.leftBackDrive.getCurrentPosition(),
                    robot.rightBackDrive.getCurrentPosition()
            );
            telemetry.update();
        }
    }
}
