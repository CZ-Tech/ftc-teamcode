package org.firstinspires.ftc.teamcode.opmode.auto.archive;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;

@Disabled
//@Config
@Autonomous(name = "AutoOrigin", group = "Auto")
public class AutoOrigin extends LinearOpMode {
    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    public static int SLEEPTIME = 245;
    public static double DRIVESPEED = 0.7;//驾驶速度
    public static double SPEDIS = 55;//推动Sample的距离
    public static double POS1 = 10;
    public static double POS2 = 30;
    public static double POS3 = -30;
    public static double POS4 = 20;
    public static  double POS5 = 9;
    public static double POS6 = 8;
    public static long TIME1 = 400;
    public static long TIME2 = 1500;
    public static long TIME3 = 1500;
    public static long TIME4 = 1500;
    public static double SPI2DIVE = -24;
    public static double SPI2DIVEANG = 60;


    @Override
    public void runOpMode() {

        // region Initialization
        runtime.reset();
        robot.init(this);
        robot.opModeState= OpModeState.Auto;
        sleep(1000);
        robot.drivetrain.resetYaw();
        telemetry.addData(">", "Hardware Initialized");

        while (opModeInInit()) {
            robot.drivetrain.getHeading();

        }
        // Wait for the 'Start' button pressed
        waitForStart();
        // endregion
        // Main Method
//        robot.subsystem.slamDunker.grab();
//
//        //预载目标
//        robot.syncRun(
//                () -> {
//                    sleep(300);
//                    robot.subsystem.slamDunker.onGround();
//                },
//                () -> {
//                    sleep(1500);
//                    robot.subsystem.slamDunker.onChamber(0.8);
//                    sleep(100);
//                    robot.subsystem.slamDunker.release();
//                    sleep(50);
//                    robot.subsystem.slamDunker.onGround();
//
//                });
//        robot.drivetrain
//                .driveStrafe(-42, 0, DRIVESPEED)
//                .sleep(400)//第一个目标
//        ;

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
//                //移动至潜水器挂样本
//                x = spline_get(spline_fit(-5, 50, -37, 0), (now - 16) / 4);
//                y = spline_get(spline_fit(44, 0, 0, 0), (now - 16) / 4);
//            } else if (now < 21) {
//                //等待挂好样本
//                x = spline_get(spline_fit(-37, 0, -37, 0), (now - 20) / 1);
//                y = spline_get(spline_fit(44, 0, 44, 0), (now - 20) / 1);
//            }

        robot.pause();

//        robot.odoDrivetrain
//                .driveStrafe(POS1, DRIVESPEED)//平移出潜水区
//                .sleep(SLEEPTIME);
//        robot.syncRun(() ->
//                robot.subsystem.slamDunker.onChamber()
//        );
//        robot.odoDrivetrain.driveStraight(POS2, DRIVESPEED)//直行
//                .sleep(SLEEPTIME)
//                .driveStrafe(POS3, DRIVESPEED)//向前走
//                .sleep(SLEEPTIME)
//        ;
//        robot.pause();
//
//        robot.odoDrivetrain
//                //开始推Sample
//                .driveStraight(POS5, DRIVESPEED)//平移调位
//                .sleep(100)
//                .driveStrafe(SPEDIS, DRIVESPEED)//推Sample到观察区
//                .sleep(100)
//        ;
//        robot.odoDrivetrain
//                .driveStrafe(-20, 0, DRIVESPEED)//回到原处
//                .sleep(100)
//        ;
//        robot.syncRun(() -> robot.subsystem.slamDunker.onGround());
//        sleep(1000);
//        robot.pause();
//
//        robot.odoDrivetrain
//                .driveStrafe(20, DRIVESPEED)
//                .sleep(125)
//        ;
//        robot.subsystem.slamDunker.grab();
//        robot.pause();
//
//        robot.odoDrivetrain
//                .driveStraighfe(SPI2DIVE, SPI2DIVEANG, DRIVESPEED)
//                ;
//        robot.drivetrain
//                .driveStrafe(-10, 0, DRIVESPEED);
//
//
//        robot.pause();
//        robot.syncRun(() -> robot.subsystem.slamDunker.onGround(DcMotor.ZeroPowerBehavior.BRAKE),
//                () -> robot.subsystem.slamDunker.release());
//        robot.odoDrivetrain
//                .driveStraighfe(-SPI2DIVE, SPI2DIVEANG, DRIVESPEED)
//        ;
//
//        sleep(850);
//        robot.subsystem.slamDunker.grab();
//        sleep(100);
//
//        robot.pause();
//        robot.odoDrivetrain
//                .driveStraighfe(SPI2DIVE + 5, SPI2DIVEANG, DRIVESPEED)
//        ;
//        robot.drivetrain
//                .driveStrafe(-10, 0, DRIVESPEED);
        //第二个
        /*
        robot.drivetrain
                .driveStraight(POS6, 0, DRIVESPEED)
                .sleep(100)
        ;
        robot.drivetrain
                .driveStrafe(SPEDIS, 0, DRIVESPEED)
                .sleep(100)
        ;
        robot.syncRun(() -> {
            sleep(200);
            robot.subsystem.slamDunker.onGround(DcMotor.ZeroPowerBehavior.BRAKE);
        });
        robot.drivetrain
                .driveStrafe(-30, 0, DRIVESPEED)
                .sleep(1000)
        ;
        robot.drivetrain
                .driveStrafe(POS4, 0, DRIVESPEED)
                .sleep(200)
        ;
        robot.subsystem.slamDunker.grab();
        sleep(200);
        robot.drivetrain
                .driveStraight(-30, 0, DRIVESPEED)
                .sleep(100)
        ;

        robot.pause();
        //运送到悬挂处
        robot.drivetrain.driveStraight(-29, 0, DRIVESPEED)
                .driveStrafe(-27, 0, DRIVESPEED)
        ;

        while (!gamepad1.a && opModeIsActive());
        //返回到观察区
        robot.drivetrain
                .driveStraight(-4,-90,DRIVESPEED)
                .sleep(SLEEPTIME)
                .turnToHeading(0,DRIVESPEED)
                .driveStraight(15,0,DRIVESPEED)
                .sleep(SLEEPTIME)
        ;
        //第二个
        robot.drivetrain
                .driveStrafe(-15,0,DRIVESPEED)//对准潜水区
                .sleep(SLEEPTIME)
                .driveStraight(8,-90,DRIVESPEED)//往上靠
                ;

        */

        while (opModeIsActive()) {
            robot.drivetrain.getHeading();
            telemetry.update();
        }

    }

    // region 统一入口
    //Red alliance

    @Disabled
    @Autonomous(name = "Auto🔴", group = "Auto", preselectTeleOp = "Duo🔴")
    public static class AutoRed extends AutoOrigin {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    //Blue alliance
    @Disabled
    @Autonomous(name = "Auto🔵", group = "Auto", preselectTeleOp = "Duo🔵")
    public static class AutoBlue extends AutoOrigin {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            super.runOpMode();
        }
    }

    // endregion
}



