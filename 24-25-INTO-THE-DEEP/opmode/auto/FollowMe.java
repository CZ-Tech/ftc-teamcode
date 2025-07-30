package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.PID;
import org.firstinspires.ftc.teamcode.common.util.status.GrabberStat;
import org.firstinspires.ftc.teamcode.common.vision.processor.ColorLocatorProcessor;
import org.firstinspires.ftc.teamcode.common.vision.processor.ColorRange;
import org.opencv.core.RotatedRect;

@Disabled
@TeleOp(name = "Follow Me")
public class FollowMe extends LinearOpMode {
    public static double Kp = 0.00342;
    public static double Ki = 0.008142;
    public static double Kd = 0.0003591;
    public static double Kf = 0.15;
    public static double integralSumLimit = Double.POSITIVE_INFINITY;
    public static double RANGE = 0.3;
    Robot robot = new Robot();
    private RotatedRect rect;
    private double realAngel;
    private double reference = 120;
    private double error;
    private double integralSum = 0;
    private double derivative;
    private double lastError = 0;
    private ElapsedTime timer;
    private PID pid;
    @Override
    public void runOpMode() {
        robot.init(this);
        Globals.targetColor = ColorRange.BLUE;
        ColorLocatorProcessor colorLocatorProcessor = new ColorLocatorProcessor(telemetry);
        robot.vision.init(colorLocatorProcessor);

        robot.command.IngrabberInit();
        robot.subsystem.grabber.grab();
        robot.subsystem.grabber.stat = GrabberStat.GRAB;

        robot.subsystem.newclaw.observer();
        pid=new PID(Kp,Ki,Kd,120);
        timer = new ElapsedTime();

        while (opModeInInit() || opModeIsActive()) {
            // 紧急停止
            if (gamepad1.touchpad) break;

            robot.gamepad1.update()
                    .keyDown("dpad_left", () -> Kp += 0.0001 * gamepad1.right_stick_y)
                    .keyDown("dpad_up", () -> Ki += 0.0001 * gamepad1.right_stick_y)
                    .keyDown("dpad_right", () -> Kd += 0.0001 * gamepad1.right_stick_y)
                    .keyDown("dpad_down", () -> integralSumLimit += 0.0001 * gamepad1.right_stick_y)
            ;
            telemetry.addData("操作方式", "按住十字键，推动右摇杆增减数值。按触摸板急停。");
            telemetry.addData("按左 Kp", Kp);
            telemetry.addData("按上 Ki", Ki);
            telemetry.addData("按右 Kd", Kd);
            telemetry.addData("按下（暂时可以不调） integralSumLimit", integralSumLimit);

            rect = colorLocatorProcessor.getRect();
            if (rect != null) {
                realAngel = rect.size.width > rect.size.height ?
                        rect.angle - 90
                        : rect.angle;

                telemetry.addData("rect x", rect.center.x);
                telemetry.addData("rect y", rect.center.y);
                telemetry.addData("rect w", rect.size.width);
                telemetry.addData("rect h", rect.size.height);
                telemetry.addData("angle", rect.angle);
                telemetry.addData("real angle", realAngel);

                robot.odoDrivetrain.driveRobot(0, pid.getResult(rect.center.x), 0);
            } else robot.odoDrivetrain.stopMotor();
            telemetry.update();
        }
    }

    private double pid(double x) {
        // calculate the error
        error = x - reference;

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        // set a limit on our integral sum
        integralSum = Range.clip(integralSum, -integralSumLimit, integralSumLimit);

        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        lastError = error;

        // reset the timer for next time
        timer.reset();
        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }
}

