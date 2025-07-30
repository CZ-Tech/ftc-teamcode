package org.firstinspires.ftc.teamcode.opmode.auto.archive;

import static org.firstinspires.ftc.teamcode.common.Globals.ODO_COUNTS_PER_INCH;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;
@Disabled
//@Config
@Autonomous(name = "TestTrajectoryQ5", preselectTeleOp = "Duo")
public class TestTrajectoryQ5 extends LinearOpMode {
    //region 初始化
    public static double time = 5;
    public static double SPEED = 0.5;
    public static double P_DRIVE_TURN_GAIN = 0.002;
    public static double P_DRIVE_STRAIGHT_GAIN = 0.1;
    public static int RUNTIME = 2;
    public static double[][] trajectory = {//坐标参数
            // t,x,y,vx,vy
            {0, 0, 0, 0, 0}, // 初始位置
            {2, 0, -12, 0, 0}, //平移到
            {2, 0, -12, 0, 50}, //等待
            {5, -54, 4, 0, 0}, //挂上预载标本
            {5, -54, 4, 0, -50},
            {8, -5, -10, 0, 0},
            {10, -50, -6, 0, 0},
            {10, -50, -6, 0, -60},
            {13, -5, -18, 0, 0},
            {15, -54, -18, 0, 0},
            {15, -54, -18, 0, -25},
            {18, -5, -24, 0, 0},
            {18, -5, -24, -50, 50},
            {23, -58, 10, 0, 0}
//            {9, -48, 44, 100, 0}, //走到推样本的位置 即样本后方
//            {12, -5, 44, 0, 0},//推第一个
//            {13, -30, 44, 0, 0},//向后走一点
//            {16, -30, 44, 0, 0},//等待标本制作完成
//            {17, -10, 44, 0, 0},//向前走 准备夹标本
//            {18, -10, 44, 0, 0},//等待夹住样本
//            {18, -10, 44, 50, 0},//夹住标本
//            {22, -40, -8, 0, 0},//移动至潜水器挂样本
//            {23, -40, -8, 0, 0},//等待挂好样本
//            {27, 0, 48, 0, 0}// 回到角落

//            ,
//            {2, 0, 0, -36, 0, 0, 0, 0, 0},
//            {4, -36, 0, -36, 0, 0, 0, 0, 0},
//            {9, -36, 150, -48, 100, 0, 80, 44, 0},
//            {12, -48, 100, -5, 0, 44, 0, 44, 0},
//            {13, -5, 0, -20, 0, 44, 0, 44, 0},
//            {14, -20, 0, -20, 0, 44, 0, 44, 0},
//            {15, -20, 0, -5, 0, 44, 0, 44, 0},
//            {16, -5, 0, -5, 0, 44, 0, 44, 0},
//            {20, -5, 50, -37, 0, 44, 0, 0, 0},
//            {21, -37, 0, -37, 0, 44, 0, 44, 0}
    };
    Robot robot = new Robot();
    ElapsedTime runtime = new ElapsedTime();
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    double x = 0;
    double y = 0;
    double now;
    double x_error, y_error;
    private double turnSpeed = 0;
    //endregion

    @Override
    public void runOpMode() {

        robot.init(this);
        robot.opModeState = OpModeState.Auto;
        packet.fieldOverlay()
                .drawImage("/dash/into-the-deep.png", 0, 0, 144, 144, Math.toRadians(-90), 72, 72, false);


        dashboard.sendTelemetryPacket(packet);
        robot.odoDrivetrain.encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDrivetrain.encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDrivetrain.encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.subsystem.slamDunker.grab();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            now = runtime.seconds();

            //region 运动模块
            for (int i = 1; i < trajectory.length; i++) {
                if (trajectory[i - 1][0] < now && now < trajectory[i][0]) {
                    x = spline_get(spline_fit(
                            trajectory[i - 1][1],
                            trajectory[i - 1][3],
                            trajectory[i][1],
                            trajectory[i][3]
                    ), (now - trajectory[i - 1][0]) / (trajectory[i][0] - trajectory[i - 1][0]));
                    y = spline_get(spline_fit(
                            trajectory[i - 1][2],
                            trajectory[i - 1][4],
                            trajectory[i][2],
                            trajectory[i][4]
                    ), (now - trajectory[i - 1][0]) / (trajectory[i][0] - trajectory[i - 1][0]));
                    break;
                }
            }
            //endregion

            x_error = x - (double) robot.odoDrivetrain.encoderCenter.getCurrentPosition() / ODO_COUNTS_PER_INCH;
            y_error = y - (double) robot.odoDrivetrain.encoderLeft.getCurrentPosition() / ODO_COUNTS_PER_INCH;
            turnSpeed = Range.clip((robot.odoDrivetrain.encoderRight.getCurrentPosition() - robot.odoDrivetrain.encoderLeft.getCurrentPosition()) * SPEED * P_DRIVE_TURN_GAIN, -1, 1);

            if (!Globals.DEBUG)
                robot.odoDrivetrain.driveRobot(y_error * P_DRIVE_STRAIGHT_GAIN * SPEED, x_error * P_DRIVE_STRAIGHT_GAIN * SPEED, turnSpeed);

            packet.fieldOverlay().setRotation(-Math.toRadians(90)).setTranslation(-40, -6 * 12 + 7).setStroke("blue").setStrokeWidth(1).strokeRect(x - 7, y - 8, 14, 16);
//            packet.fieldOverlay().setFill("red").fillRect(
//                    (double) robot.odoDrivetrain.encoderCenter.getCurrentPosition() / 2000 * Math.PI * 1.88976378-7,
//                    (double) robot.odoDrivetrain.encoderLeft.getCurrentPosition() / 2000 * Math.PI * 1.88976378-8,
//                    14,16);
            dashboard.sendTelemetryPacket(packet);
            if (Globals.DEBUG) sleep(200);
//            robot.telemetry.addData("x", x);
//            robot.telemetry.addData("y", y);
//            robot.telemetry.update();

        }
    }

    public double[] spline_fit(double x0, double dx0, double x1, double dx1) {
        double a = 2 * x0 + dx0 - 2 * x1 + dx1;
        double b = -3 * x0 - 2 * dx0 + 3 * x1 - dx1;
        double c = dx0;
        double d = x0;
        return new double[]{a, b, c, d};
    }


    public double spline_get(double[] spline, double u) {
        return spline[0] * u * u * u + spline[1] * u * u + spline[2] * u + spline[3];
    }
}
