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
@Autonomous(name = "TestTrajectoryQ9", preselectTeleOp = "Duo")
public class testTrajectoryQ9 extends LinearOpMode {
    //region 初始化
    public double time = 5;
    public double SPEED = 0.5;
    public double P_DRIVE_TURN_GAIN = 0.002;
    public double P_DRIVE_STRAIGHT_GAIN = 0.1;
    public int RUNTIME = 2;
    public double[][] trajectory = {//坐标参数
                                           // t,x,y,vx,vy
                                           {0, 0, 0, 0, 0}, // 初始位置
                                           {5, 0, 0, 0, 0}, // 初始位置
                                           {7, -39, 0, 0, 0}, //平移到潜水器
                                           {9, -39, 0, 0, 0}, //等待
                                           {9, -39, 0, 150, 80}, //挂上预载标本
                                           {13, -48, 44, 100, 0}, //走到推样本的位置 即样本后方
                                           {15, -5, 44, 0, 0},//推第一个
                                           {16, -30, 44, 0, 0},//向后走一点
                                           {19, -30, 44, 0, 0},//等待标本制作完成
                                           {20, -10, 44, 0, 0},//向前走 准备夹标本
                                           {21, -10, 44, 0, 0},//等待夹住样本
                                           {21, -10, 44, 50, 0},//夹住标本
                                           {25, -40, -8, 0, 0},//移动至潜水器挂样本
                                           {26, -40, -8, 0, 0},//等待挂好样本
                                           {30, 0, 48, 0, 0}// 回到角落


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
        packet.fieldOverlay().drawImage("/dash/into-the-deep.png", 0, 0, 144, 144, Math.toRadians(-90), 72, 72, false);


        dashboard.sendTelemetryPacket(packet);
        robot.odoDrivetrain.encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDrivetrain.encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDrivetrain.encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.subsystem.slamDunker.grab();
//        waitForStart();
//        runtime.reset();
//        //region 机械臂模块
//        if (!Globals.DEBUG) robot.syncRun(() -> {
//            robot.subsystem.slamDunker.grab();//先夹紧
//            sleep(5800);
//            robot.subsystem.slamDunker.onAir();//放下机械臂
//            sleep(700);
//            robot.subsystem.slamDunker.onChamber(0.8);//扣预载样本
//            sleep(400);
//            robot.subsystem.slamDunker.release();//松开爪子
//            sleep(200);
//            robot.subsystem.slamDunker.onAir();//放下机械臂
//        }, () -> {
//            sleep(11000);
//            robot.subsystem.slamDunker.onChamber();//挂上机械臂
//        }, () -> {
//            sleep(17500);
//            robot.subsystem.slamDunker.onGround(DcMotor.ZeroPowerBehavior.BRAKE).release();//将机械臂放到地上并松开
//        }, () -> {
//            sleep(20500);
//            robot.subsystem.slamDunker.grab();//抓住人类玩家做好的样本
//        }, () -> {
//            sleep(25500);
//            robot.subsystem.slamDunker.onChamber(0.8);//扣第二个样本
//            sleep(500);
//            robot.subsystem.slamDunker.release();//松开爪子
//            sleep(200);
//            robot.subsystem.slamDunker.onAir();//放下机械臂
//            sleep(900);
//            robot.subsystem.slamDunker.onChamber(0.3);//收回
//        });
//        //endregion
//        while (opModeIsActive()) {
//            now = runtime.seconds();
//
//            //region 运动模块
//            for (int i = 1; i < trajectory.length; i++) {
//                if (trajectory[i - 1][0] < now && now < trajectory[i][0]) {
//                    x = spline_get(spline_fit(trajectory[i - 1][1], trajectory[i - 1][3], trajectory[i][1], trajectory[i][3]), (now - trajectory[i - 1][0]) / (trajectory[i][0] - trajectory[i - 1][0]));
//                    y = spline_get(spline_fit(trajectory[i - 1][2], trajectory[i - 1][4], trajectory[i][2], trajectory[i][4]), (now - trajectory[i - 1][0]) / (trajectory[i][0] - trajectory[i - 1][0]));
//                    break;
//                }
//            }
            //endregion
//            if (now < 2) {
//                //平移到潜水器
//                x = spline_get(spline_fit(0, 0, -37, 0), now / 2);
//                y = spline_get(spline_fit(0, 0, 0, 0), now / 2);
//            } else if (now < 4) {
//                //等待挂上预载标本
//                x = spline_get(spline_fit(-37, 0, -37, 0), (now - 2) / 2);
//                y = spline_get(spline_fit(0, 0, 0, 0), (now - 2) / 2);
//            } else if (now < 9) {
//                //走到推样本的位置 即样本后方
//                x = spline_get(spline_fit(-37, 150, -48, 100), (now - 4) / 5);
//                y = spline_get(spline_fit(0, 50, 44, 0), (now - 4) / 5);
//            } else if (now < 12) {
//                //推第一个
//                x = spline_get(spline_fit(-48, 100, -5, 0), (now - 9) / 3);
//                y = spline_get(spline_fit(44, 0, 44, 0), (now - 9) / 3);
//            } else if (now < 13) {
//                //向后走一点
//                x = spline_get(spline_fit(-5, 0, -20, 0), (now - 12) / 1);
//                y = spline_get(spline_fit(44, 0, 44, 0), (now - 12) / 1);
//            } else if (now < 14) {
//                //等待标本制作完成
//                x = spline_get(spline_fit(-20, 0, -20, 0), (now - 13) / 1);
//                y = spline_get(spline_fit(44, 0, 44, 0), (now - 13) / 1);
//            } else if (now < 15) {
//                //向前走 准备夹标本
//                x = spline_get(spline_fit(-20, 0, -5, 0), (now - 14) / 1);
//                y = spline_get(spline_fit(44, 0, 44, 0), (now - 14) / 1);
//            } else if (now < 16) {
//                //等待夹住标本
//                x = spline_get(spline_fit(-5, 0, -5, 0), (now - 15) / 1);
//                y = spline_get(spline_fit(44, 0, 44, 0), (now - 15) / 1);
//            } else if (now < 20) {
//                //移动至潜水器挂样本1
//                x = spline_get(spline_fit(-5, 50, -37, 0), (now - 16) / 4);
//                y = spline_get(spline_fit(44, 0, 0, 0), (now - 16) / 4);
//            } else if (now < 21) {
//                //等待挂好样本
//                x = spline_get(spline_fit(-37, 0, -37, 0), (now - 20) / 1);
//                y = spline_get(spline_fit(44, 0, 44, 0), (now - 20) / 1);
//            }


            x_error = x - (double) robot.odoDrivetrain.encoderCenter.getCurrentPosition() / ODO_COUNTS_PER_INCH;
            y_error = y - (double) robot.odoDrivetrain.encoderLeft.getCurrentPosition() / ODO_COUNTS_PER_INCH;
            turnSpeed = Range.clip((robot.odoDrivetrain.encoderRight.getCurrentPosition() - robot.odoDrivetrain.encoderLeft.getCurrentPosition()) * SPEED * P_DRIVE_TURN_GAIN, -1, 1);

            if (!Globals.DEBUG)
                robot.odoDrivetrain.driveRobot(y_error * P_DRIVE_STRAIGHT_GAIN * SPEED, x_error * P_DRIVE_STRAIGHT_GAIN * SPEED, turnSpeed);

            packet.fieldOverlay().setRotation(-Math.toRadians(90)).setTranslation(4, -6 * 12 + 7).setStroke("blue").setStrokeWidth(1).strokeRect(x - 7, y - 8, 14, 16);
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

//    public double[] spline_fit(double x0, double dx0, double x1, double dx1) {
//        double a = 2 * x0 + dx0 - 2 * x1 + dx1;
//        double b = -3 * x0 - 2 * dx0 + 3 * x1 - dx1;
//        double c = dx0;
//        double d = x0;
//        return new double[]{a, b, c, d};
//    }
//
//
//    public double spline_get(double[] spline, double u) {
//        return spline[0] * u * u * u + spline[1] * u * u + spline[2] * u + spline[3];
//    }

