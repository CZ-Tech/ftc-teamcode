package org.firstinspires.ftc.teamcode.opmode.route;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;

public class B23 {
    public static void run(RobotHardware robot) {
        //TODO:待调试
        robot.driveSpeed = 0.7;
        robot.driveStraight(9, 0, 0.5);
//        robot.holdHeading(-45,0.3,0.5);
        robot.driveStraight(22, -45, 0.5);
        //启动飞轮放像素
        robot.setLeftIntakePower(-0.7)
                .driveStraight(-3.5, -45, 0.5)
                .setLeftIntakePower(0)
                .sleep(500);
        robot

                .turnToHeading(-90, 0.5)
                .driveStraight(-5,-90,0.3)
                .driveStrafe(-20, -90, 0.4);

//        while (robot.myOpMode.opModeIsActive() && robot.getHeading() > -115)
//            robot.setDrivePower(-0.15, 0.15, 0.6, -0.6);
//        robot.stopMotor();


        // AprilTag 图像识别开始

//        robot.initAprilTagVision();
//        robot.setManualExposure(4, 250);
        robot.holdHeading(-90, 0.3,1);
        //已修改为正确的编号
        //TODO：未调试
        robot.driveToAprilTag(9,21,-29,-90,0.3);
//        robot.closeVision();
        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT1);
        robot.turnToHeading(-90,0.5);
        // April tag 图像识别结束
        double st = 0;//单位s,提供50hz的舵机基准信号
        while (st < 0.5) {
            robot.setIntakeRoller(IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }//启动时信号要在中位

        robot.holdHeading(-90, 0.3,1);
        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
        robot.driveStraight(2., -90, 0.3);
        st = 0;
        while (st < 0.6) {
            robot.setIntakeRoller(IntakeRollerPosition.IN);
            robot.sleep(19);
            st = st + 0.02;
        }
        st = 0;
        while (st < 0.1) {
            robot.setIntakeRoller(IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }
        robot.setIntakeFrontPosition(IntakeFrontPosition.IN);
        robot.s(2);
        robot
                //.driveStrafe(-13, -90, 0.5)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
                .turnToHeading(90, 0.5)
                .driveStraight(75, 90, 1)
                .driveStrafe(-32, 90, 0.5)
                .holdHeading(90, 0.3,1)
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD);
//                .initAprilTagVision()
//                .setManualExposure(4, 250);
        robot.driveToAprilTag(3,10,-3,90,0.3);
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 3.75) {
//            robot.driveRobot(Range.clip(0.2 * (d - 3.75), 0, 0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
        robot.closeVision();
        robot
                .holdHeading(90, 0.3,1)
                .setIntakeBackPosition(IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.TOP)
                .sleep(500)
                .driveStrafe(-7, 90, 0.3)
                .sleep(500)
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .sleep(1000)
                .setIntakeFrontPosition(IntakeFrontPosition.OUT)
                .setIntakeBackPosition(IntakeBackPosition.IN)
                .sleep(500)
                .setIntakeBackPosition(IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
        ;

        robot.turnToHeading(0, 0.3);
        robot.holdHeading(0,0.3,1000);
        robot.resetYaw();
//        robot
//                .driveStraight(0.5,24.0,-90)
//                //向右至像素
//                .driveStrafe(0.5,14.0,-90)
//                //启动飞轮放像素
//                .setLeftIntakePower(-1.0)
//                .sleep(500)
//                .setLeftIntakePower(0)
//                //向后直走回到初始位置
//                .driveStraight(0.5,24.0,-90)
//                //平移至板前
//                .G(0)
//                .G(90)
//                .driveStrafe(0.5,84.0,90)
//                .driveStraight(0.5,-24.0,90)
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
//                .driveStraight(0.5,28.0,90)
//                .driveStrafe(0.5,12.0,90)
//        ;
    }
}
