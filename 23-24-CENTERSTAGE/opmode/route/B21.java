package org.firstinspires.ftc.teamcode.opmode.route;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;

//@Config
public class B21 {
    public static double DISTANCE_X = -4.5;
    public static double DISTANCE_Y = 20.5;
    public static double BOARD_DISTANCE_X = -5.5;
    public static double BOARD_DISTANCE_Y = 10;

    public static void run(RobotHardware robot) {
        //向前直走
        robot.driveSpeed = 0.3;
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
        robot.resetYaw();
        robot.driveStraight(15, 0, 0.5);
        robot.driveStraight(16, 45, 0.5);

//                        //启动飞轮放像素
        robot.setLeftIntakePower(-0.5)
                .s(1)
                .setLeftIntakePower(0)
                .sleep(400);
        list+="\n放置紫像素："+runtime.seconds();
//                        //边走边转
//                .driveStrafe(0.3, -30.0, 0)
//                .turnToHeading(0.3, 90)
//                .driveStraight(0.3, -8.0, 90)
//                .driveStrafe(0.3, 34.0, 90)

        //向后直走回到初始位置

        robot.s(4.0)
                //向前走到中门
                .turnToHeading(-90.0,0.5)
                .driveStrafe(-8,-90,0.3);

        // April tag 图像识别开始
//        robot.initAprilTagVision();
//        robot.setManualExposure(4, 250);

//        while (!robot.myOpMode.isStopRequested() && desiredTag == null) {
//            desiredTag = robot.getAprilTag(8);
//        }

//        robot.initAprilTagVision();
//        robot.setManualExposure(6, 250);
//        AprilTagDetection desiredTag = null;
//        while (!robot.myOpMode.isStopRequested() && desiredTag == null) {
//            desiredTag = robot.getAprilTag(8);
//        }
//        robot.sleep(30000);
        robot.turnToHeading(-90,0.5);
        robot.setIntakeArmPosition(IntakeArmPosition.TOP);
        //已修改为正确的编号
        //TODO：未调试
//        robot.driveToAprilTag(9,20.5,-4.5,-90);
        robot.driveToAprilTag(9,DISTANCE_Y,DISTANCE_X,-90);
//        robot.closeVision();
//        // April tag 图像识别结束
        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT1);
        robot.turnToHeading(-90, 0.5);
        double st = 0;//单位s,提供50hz的舵机基准信号
        while (st < 0.5) {
            robot.setIntakeRoller(IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }//启动时信号要在中位
        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
        robot.driveStraight(1.5,-90,0.5);
        st = 0;
        while (st < 1) {
            robot.setIntakeRoller(IntakeRollerPosition.SLOWIN);
            robot.sleep(19);
            st = st + 0.02;
        }
        robot.setIntakeFrontPosition(IntakeFrontPosition.IN);
        robot.s(5);
        robot.setIntakeArmPosition(IntakeArmPosition.BACK);
        st = 0;
        while (st < 0.1) {
            robot.setIntakeRoller(IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }
        list+="\n拾取堆放区像素："+runtime.seconds();
        robot
//                .turnToHeading(0)
//                .rush(25,0)
                .driveStrafe(-22, -90, 0.8)
                .turnToHeading(90, 0.8)
                .rush(78, 90)
                .driveStrafe(-32, 90, 0.8)
                .holdHeading(90,0.3,1)
        .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD);
//                .initAprilTagVision()
//                .setManualExposure(5, 250);
//        AprilTagDetection desiredTag = null;
////        desiredTag = null;
//        while (!robot.myOpMode.isStopRequested() && desiredTag == null) {
//            desiredTag = robot.getAprilTag(1);
//        }
//
//        robot.translateToAprilTag(
//                desiredTag,
//                14,
//                -5,
//                90
//        );
        robot.driveToAprilTag(1,BOARD_DISTANCE_Y,BOARD_DISTANCE_X,90);
        robot.closeVision();
        list+="\n放置黄色像素："+runtime.seconds();
        robot
                .turnToHeading(90, 0.5)
                .sleep(500)
                .setIntakeBackPosition(IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.TOP)
                .sleep(500)
                .driveStrafe(7, 90, 0.5)
                .sleep(500)
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .sleep(1000)
                .setIntakeFrontPosition(IntakeFrontPosition.OUT)
                .setIntakeBackPosition(IntakeBackPosition.IN)
                .sleep(500)
                .setIntakeBackPosition(IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
                .sleep(1000);
        list+="\n放置白色像素："+runtime.seconds();
        robot
        .turnToHeading(0,0.7)
                .holdHeading(0,0.3,1)
        ;
        list+="\n放完像素："+runtime.seconds();
        robot.myOpMode.telemetry.addData("R21",list);
        robot.myOpMode.telemetry.update();
        while (robot.myOpMode.opModeIsActive()){}


//        robot
//                .w(24)
//                .a(24)
//                .s(24)
//                .d(24)
//                .t(90)
//        ;
//        //平移至板前
//        robot.driveStrafe(0.5,84.0,90)
//                .driveStraight(0.5,-14.0,-90)
//                .driveStrafe(0.5,5.0,90)
//                //像素上板
//                .setOneTimeMotorPower(0.5)
//                .sleep(500)
//                .setOneTimeMotorPower(0.0)
//                .sleep(1000)
//                .driveStrafe(-1.0)
//                .setOneTimeMotorPower(-0.5)
//                .sleep(500)
//                .setOneTimeMotorPower(0.0)
//                //停回后台
//                .driveStraight(0.5,20.0,90)
//                .driveStrafe(0.5,12.0,90)
//        ;
    }
}
