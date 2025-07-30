//package org.firstinspires.ftc.teamcode.opmode.auto;
//
//import static org.firstinspires.ftc.teamcode.common.Globals.ODO_COUNTS_PER_INCH;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.common.Globals;
//import org.firstinspires.ftc.teamcode.common.Robot;
//import org.firstinspires.ftc.teamcode.common.util.OpModeState;
//import org.firstinspires.ftc.teamcode.common.drive.Trajectory;
//
//@Config
//@Disabled
//@Autonomous(name = "TestTrajectoryQ38", preselectTeleOp = "Duo")
//public class TestTrajectoryQ38 extends LinearOpMode {
//    //region 初始化
//    public static double time = 5;
//    public static double SPEED = 0.5;
//    public static double P_DRIVE_TURN_GAIN = 0.002;
//    public static double P_DRIVE_STRAIGHT_GAIN = 0.1;
//    public static int RUNTIME = 2;
//    Robot robot = new Robot();
//    public Trajectory[] trajectory = {
//            new Trajectory(0, 0, 0, 0, 0, () -> {
//
//            }),
//            new Trajectory(1, -37, 0, 0, 0), // 初始位置
//            new Trajectory(3, -37, 0, 0, 0), //平移到潜水器
//            new Trajectory(3, -37, 0, 150, 80), //等待
//            new Trajectory(6, -48, 44, 100, 0), //样本后方
//            new Trajectory(8, -5, 44, 0, 0,() -> {}), //推第一个样本
//            new Trajectory(9, -30, 44, 0, 0), //往后退
//            new Trajectory(11, -30, 44, 0, 0), //等待夹上
////            new Trajectory(11, -30, 44, 75, -50),//移动到潜水器前的初速度
//            new Trajectory(12, -8, 44, 0, 0),//上前到位
//            new Trajectory(13, -8, 44, 0, 0),//等待夹上标本
//            new Trajectory(13, -8, 44, 50, -100),//移到潜水器前的初速度
//            new Trajectory(15, -38, -4, 0, 0), //移到潜水器
//            new Trajectory(17, -38, -4, 0, 0),//挂完样本
//            new Trajectory(17, -38, -4, 150, 80),//离开潜水器的初速度
//            new Trajectory(19, -20, 44, 0, 0),//到达待抓取状态
//            new Trajectory(20,-20,44,0,0),//等待人类玩家确定样本位置
//            new Trajectory(21,-5,44,0,0),//上前夹取
//            new Trajectory(23,-5,44,0,0),//夹上
//            new Trajectory(23,-5,44,50,-100),
//            new Trajectory(25,-39,-7,-0,0),//回到潜水区
//            new Trajectory(27,-39,-7,-0,0),//等待挂好
//            new Trajectory(29,0,40,-0,0)
//
//
//
//
//
//
//    };
//    ElapsedTime runtime = new ElapsedTime();
//    TelemetryPacket packet = new TelemetryPacket();
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    double x = 0;
//    double y = 0;
//    double now;
//    double x_error, y_error;
//    private double turnSpeed = 0;
//    //endregion
//
//    @Override
//    public void runOpMode() {
//
//        robot.init(this);
//        robot.opModeState = OpModeState.Auto;
//        packet.fieldOverlay()
//                .drawImage("/dash/into-the-deep.png", 0, 0, 144, 144, Math.toRadians(-90), 72, 72, false);
//
//
//        dashboard.sendTelemetryPacket(packet);
//        robot.odoDrivetrain.encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.odoDrivetrain.encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.odoDrivetrain.encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.subsystem.slamDunker.grab();
//        waitForStart();
//        runtime.reset();
//
////        if(!Globals.DEBUG) for (int i = 1; i < trajectory.length; i++){
////            final int index = i;
////            new Thread(() -> {
////                sleep(trajectory[index].t*1000);
////                trajectory[index].func.run();
////            }).start();
////        }
//
//        if (!Globals.DEBUG) robot.syncRun(() -> {
//            robot.subsystem.slamDunker.grab();//先夹紧
//            sleep(300);
//            robot.subsystem.slamDunker.onAir();//放下机械臂
//            sleep(500);
//            robot.subsystem.slamDunker.onChamber(1);//扣预载样本
//            sleep(400);
//            robot.subsystem.slamDunker.release();//松开爪子
//            sleep(100);
//            robot.subsystem.slamDunker.onAir();//放下机械臂
//        },
//        () -> {
//            sleep(4000);
//            robot.subsystem.slamDunker.onChamber(0.3);
//            sleep(100);
//            robot.subsystem.slamDunker.release();
//        },
//        () -> {
//            sleep(10000);
//            robot.subsystem.slamDunker.onGround(DcMotor.ZeroPowerBehavior.BRAKE);
//        },
//        () -> {
//            sleep(12500);
//            robot.subsystem.slamDunker.grab();
//        },
//        () -> {
//            sleep(15000);
//            robot.subsystem.slamDunker.onChamber(0.9);
//            sleep(400);
//            robot.subsystem.slamDunker.release();
//            sleep(200);
//            robot.subsystem.slamDunker.onAir();
//        },
//        () -> {
//            sleep(21500);
//            robot.subsystem.slamDunker.grab();
//
//        },
//        () -> {
//            sleep(25500);
//            robot.subsystem.slamDunker.onChamber(0.9);
//            sleep(400);
//            robot.subsystem.slamDunker.release();
//            sleep(200);
//            robot.subsystem.slamDunker.onAir();
//            sleep(500);
//            robot.subsystem.slamDunker.onChamber(0.5);
//        });
//        while (opModeIsActive()) {
//            now = runtime.seconds();
//
//            //region 运动模块
//            for (int i = 1; i < trajectory.length; i++) {
//                if (trajectory[i-1].t < now && now < trajectory[i].t) {
//                    x = spline_get(spline_fit(
//                            trajectory[i - 1].x,
//                            trajectory[i - 1].dx,
//                            trajectory[i].x,
//                            trajectory[i].dx
//                    ), (now - trajectory[i - 1].t) / (trajectory[i].t - trajectory[i - 1].t));
//                    y = spline_get(spline_fit(
//                            trajectory[i - 1].y,
//                            trajectory[i - 1].dy,
//                            trajectory[i].y,
//                            trajectory[i].dy
//                    ), (now - trajectory[i - 1].t) / (trajectory[i].t - trajectory[i - 1].t));
//                    break;
//                }
//            }
//
//            //endregion
//            x_error = x - (double) robot.odoDrivetrain.encoderCenter.getCurrentPosition() / ODO_COUNTS_PER_INCH;
//            y_error = y - (double) robot.odoDrivetrain.encoderLeft.getCurrentPosition() / ODO_COUNTS_PER_INCH;
//            turnSpeed = Range.clip((robot.odoDrivetrain.encoderRight.getCurrentPosition() - robot.odoDrivetrain.encoderLeft.getCurrentPosition()) * SPEED * P_DRIVE_TURN_GAIN, -1, 1);
//
//            if (!Globals.DEBUG)
//                robot.odoDrivetrain.driveRobot(y_error * P_DRIVE_STRAIGHT_GAIN * SPEED, x_error * P_DRIVE_STRAIGHT_GAIN * SPEED, turnSpeed);
//
//            packet.fieldOverlay().setRotation(-Math.toRadians(90)).setTranslation(4, -6 * 12 + 7).setStroke("blue").setStrokeWidth(1).strokeRect(x - 7, y - 8, 14, 16);
////            packet.fieldOverlay().setFill("red").fillRect(
////                    (double) robot.odoDrivetrain.encoderCenter.getCurrentPosition() / 2000 * Math.PI * 1.88976378-7,
////                    (double) robot.odoDrivetrain.encoderLeft.getCurrentPosition() / 2000 * Math.PI * 1.88976378-8,
////                    14,16);
//            dashboard.sendTelemetryPacket(packet);
//            if (Globals.DEBUG) sleep(200);
////            robot.telemetry.addData("x", x);
////            robot.telemetry.addData("y", y);
////            robot.telemetry.update();
//
//        }
//    }
//
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
//}
