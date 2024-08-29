package org.firstinspires.ftc.teamcode.opmode.route;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.test.PutThePixel;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;




import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;




public class B12 {


    public static void run(RobotHardware robot) {
        ElapsedTime runtime = new ElapsedTime();
        String list = "";
        runtime.reset();
       //+0
        robot
//                .turnToHeading(90, 0.5)
                .resetYaw()
//                .driveStraight(24, 90, 0.5)
//                .holdHeading(90, 0.3, 1);
        ;
        robot
                //Purple Pixel
                .rush(27.5, 0)
                .setLeftIntakePower(-0.6)
                .s(3)
                .setLeftIntakePower(0);
        robot
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .turnToHeading(90, 0.7)
                .holdHeading(90, 0.3, 1)
                .driveStraight(30, 90, 0.7)
                .turnToHeading(90, 0.3)
                .driveToAprilTag(2, 11, -5, 90);
        //边转边走
//        robot.initAprilTagVision();
//        robot.setManualExposure(4, 250);
//        robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);
//        AprilTagDetection desiredTag = null;
//        while (!robot.myOpMode.isStopRequested() && desiredTag == null) {
//            desiredTag = robot.getAprilTag(2);
//        }
//        robot.translateToAprilTag(
//                desiredTag,
//                11,
//                -6,
//                90
//        );
        robot.closeVision();
//        AutoDriveToAprilTag.goToAprilTag(robot, 14, 2,1);
        list += "\n放置黄色像素：" + runtime.seconds();
        robot
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.BACK)
                .sleep(1000)
                //原回后台程序
                .driveStraight(-4, 90, 0.5)
                .driveStrafe(-21, 90, 0.5)
                .driveStraight(13.0, 90, 0.5)
                .turnToHeading(0,0.5)
                .driveStrafe(-3,90,0.3)
                .holdHeading(0,0.3,1)
        ;
//        +2
//        robot
//                紫像素
//                .rush(26, 0)
//                .setLeftIntakePower(-0.8)
//                .driveStraight(-1,0,0.5)
//                .setLeftIntakePower(0)
                //上板
//                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
//                .turnToHeading(90,0.7)
//                .rush(30,90)
//                .driveToAprilTag(2,11.6,-5,90);

//
//                robot
//                .setIntakeBackPosition(IntakeBackPosition.OUT)
//                .sleep(1000)
//                        .setIntakeArmPosition(IntakeArmPosition.BACK)
//                        .sleep(500)
//                        .driveStrafe(5,90,0.8)
//                        .turnToHeading(90,0.3)
//                        .rush(-98,90)
//                        .turnToHeading(-90,1)
//        .turnToHeading(-90,0.3);
//        robot.driveToAprilTag(9,21,-28.5,-90);
//        robot.turnToHeading(-90,0.3);

//        double st = 0;//单位s,提供50hz的舵机基准信号
//        while (st < 0.5) {
//            robot.setIntakeRoller(IntakeRollerPosition.STOP);
//            robot.sleep(19);
//            st = st + 0.02;
//        }//启动时信号要在中位
//        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
//        robot.setIntakeBackPosition(IntakeBackPosition.IN);
//        robot.w(2.5);
//        st = 0;
//        while (st < 1) {
//            robot.setIntakeRoller(IntakeRollerPosition.SLOWIN);
//            robot.sleep(19);
//            st = st + 0.02;}
//
//        robot.s(1);
//        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
//        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT2);
//        robot.w(1.2);

//        st = 0;
//        while (st < 0.8) {
//            robot.setIntakeRoller(IntakeRollerPosition.SLOWIN);
//            robot.sleep(19);
//            st = st + 0.02;}
//        st = 0;
//        while (st < 0.1) {
//            robot.setIntakeRoller(IntakeRollerPosition.STOP);
//            robot.sleep(19);
//            st = st + 0.02;
//        }
//        robot.setIntakeBackPosition(IntakeBackPosition.IN);
//        robot.setIntakeFrontPosition(IntakeFrontPosition.IN);
//        robot.setIntakeArmPosition(IntakeArmPosition.LIFTALITTLE);
//        robot
//                .sleep(500)
//                .turnToHeading(90,0.8)
//                .setIntakeArmPosition(IntakeArmPosition. BACK)
//                .turnToHeading(90,0.8)
//                .rush(80, 90)
//                .driveStrafe(-28, 90, 0.8)
//                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
//                .turnToHeading(90,0.5)
//                .driveToAprilTag(2,10.5,3,90);
//
//
//        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
//        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
//        robot.sleep(1500)
//                .setIntakeArmPosition(IntakeArmPosition.BACK)
//                .turnToHeading(0,0.7)
//                .holdHeading(0,0.3,1000);
//
//
//        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
//        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
//        robot.sleep(1500)
//                .setIntakeArmPosition(IntakeArmPosition.BACK)
//                .turnToHeading(0,0.7)
//                .holdHeading(0,0.3,1000);
        list += "\n停回后台：" + runtime.seconds();
        robot.myOpMode.telemetry.addData("B12", list);
        robot.myOpMode.telemetry.update();
        while (robot.myOpMode.opModeIsActive()) {
        }
    }
}
