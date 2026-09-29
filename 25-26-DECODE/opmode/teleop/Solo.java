package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.util.Alliance;

@Config
@Disabled
@TeleOp(name = "Solo")
public class Solo extends LinearOpMode {
    Robot robot = new Robot();
    public double initAngle = 50;

    @Override
    public void runOpMode() {
        robot.init(this);
        telemetry.addLine("Robot ready!");
        robot.gyroTracker.init(initAngle);
        robot.teamColor = Alliance.BLUE;
        waitForStart();
        robot.limelight.pipelineSwitch(robot.teamColor.getBaseAprilTag() % 20);
        robot.limelight.setPollRateHz(50);
        robot.limelight.start();

        while (opModeIsActive()) {
            // update 必须每次循环最开始执行，并且整个循环内每个手柄只能执行一次。
            robot.gamepad1.update();

            robot.odo.update();

            robot.gamepad1
                    .keyDown("a", () -> robot.command.autoSteer())
                    .keyDown("y", () -> robot.gyroTracker.init(initAngle))
            ;

            robot.gamepad1
                    .keyDown("share", "options", () -> robot.odoDrivetrain.resetYaw());

            double[] result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());


            robot.odoDrivetrain.driveRobotFieldCentricWithTeleOpHeadReset(
                    sss(-gamepad1.left_stick_y),
                    sss(gamepad1.left_stick_x),
                    sss(gamepad1.right_stick_x)
            );
//            robot.telemetry.addData("Offset", robot.gyroTracker.getCurrentOffset(UnnormalizedAngleUnit.DEGREES));
//            robot.telemetry.addData("Unnormalized Heading", robot.odoDrivetrain.getHeading(UnnormalizedAngleUnit.DEGREES));
            robot.telemetry.addData("Normalized Heading", robot.odoDrivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("ty", result[7]);
            robot.telemetry.addData("dis", result[10]);
            robot.telemetry.addData("tx", result[6]);
            robot.telemetry.addData("Pipeline", robot.limelight.getStatus().getPipelineIndex());
            robot.telemetry.addData("FPS", robot.limelight.getStatus().getFps());
            robot.telemetry.addData("Connection Info", robot.limelight.getConnectionInfo());
            robot.telemetry.update();
        }
        TaskLoopFrame.stopAndClearAll();
        robot.limelight.stop();
    }

    @Disabled
    @TeleOp(name = "Solo🔴", group = "Solo")
    public static class SoloRed extends Solo {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    @Disabled
    @TeleOp(name = "Solo🔵", group = "Solo")
    public static class SoloBlue extends Solo {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            super.runOpMode();
        }
    }

    private double sss(double v){
        if (v > 0) { //若手柄存在中位漂移或抖动就改0.01
            v = 0.8 * Math.pow(v, 7) + 0.15 * Math.pow(v, 3) + 0.05 * v + 0.09;//0.09是23-24赛季底盘启动需要的功率
        } else if (v < 0) { //若手柄存在中位漂移或抖动就改-0.01
            v = 0.8 * Math.pow(v, 7) + 0.15 * Math.pow(v, 3) + 0.05 * v - 0.09; //三次方是摇杆曲线
        } else {
            // XBOX和罗技手柄死区较大无需设置中位附近
            // 若手柄存在中位漂移或抖动就改成 v*=13
            // 这里的13是上面的0.13/0.01=13
            v = 0;
        }
        return v;
    }
}
