package org.firstinspires.ftc.teamcode.TelemetryTest;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.IntakeArmPosition;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.IntakeBackPosition;

public class R12_Test {
    //    @Override
    public static void run(RobotHardware robot) {

        robot.resetYaw();
// +2
//        robot
//                //紫像素
//                .rush(25.0, 0)
//                .setLeftIntakePower(-0.8)
//                .driveStraight(-1,0,0.8)
//                .setLeftIntakePower(0)
//                //上板
//                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
//                .turnToHeading(-90,0.7)
//                .rush(30,-90)
//                .driveToAprilTag(2,12,-4,-90);
//        //FIXME:红色2号位置应该是apriltag 5
//
//
//        robot
//                .setIntakeBackPosition(IntakeBackPosition.OUT)
//                .sleep(1000)
//                        .setIntakeArmPosition(IntakeArmPosition.BACK)
//                        .driveStrafe(4,-90,0.8)
//                        .turnToHeading(-90,0.3)
//                        .rush(-90,-90)
//                        .turnToHeading(90,1)
//        .turnToHeading(90,0.3);
//        robot.driveToAprilTag(10,18.75,3,90);
//        robot.turnToHeading(90,0.3);
//                robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT1);
//        double st = 0;//单位s,提供50hz的舵机基准信号
//        while (st < 0.5) {
//            robot.setIntakeRoller(IntakeRollerPosition.STOP);
//            robot.sleep(19);
//            st = st + 0.02;
//        }//启动时信号要在中位
//        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
//        robot.setIntakeBackPosition(IntakeBackPosition.IN);
//        st = 0;
//        while (st < 1) {
//            robot.setIntakeRoller(IntakeRollerPosition.IN);
//            robot.sleep(19);
//            st = st + 0.02;}
//
//        robot.s(1);
//        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
//        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT2);
//        robot.w(1);
//
//        st = 0;
//        while (st < 1.2) {
//            robot.setIntakeRoller(IntakeRollerPosition.IN);
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
//        robot.s(2);
//        robot.setIntakeArmPosition(IntakeArmPosition.BACK);
//        robot
//                .driveStrafe(22, 90, 0.8)
//                .turnToHeading(-90,0.8)
//                .rush(80, -90)
//                .driveStrafe(22, -90, 0.8)
//                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
//                .turnToHeading(-90,0.5)
//                .driveToAprilTag(1,9.5,-4,-90);
//        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
//        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
//        robot.sleep(1000)
//                .setIntakeArmPosition(IntakeArmPosition.BACK)
//                .sleep(1000)
//                .turnToHeading(0,0.7)
//                .holdHeading(0,0.3,1000);
//        //FIXME:红色1号位置应该是apriltag 4


        //+0 NEW
        robot
                //紫像素
                .rush(25.0, 0)
                .setLeftIntakePower(-0.8)
                .driveStraight(-1, 0, 0.8)
                .setLeftIntakePower(0)
                //上板
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .turnToHeading(-90, 0.7)
                .driveStraight(30, -90, 0.7)
                .driveToAprilTag(2, 15, -4.25, -90, 0.3);
//        robot.driveRobot(0.3, 0, 0);
        //FIXME:红色2号位置应该是apriltag 5

        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
        while (d > 3) {
            robot.driveRobot(Range.clip(0.2*(d-3),0,0.3), 0, 0);
            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
        }
        robot.stopMotor();

        robot
                .setIntakeBackPosition(IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
                .driveStrafe(27, -90, 0.5)
                .turnToHeading(0, 0.5)
                .holdHeading(0, 0.3, 2);


// +0old
//        robot
//                //向前直走
//                .driveStraight(28.0, 0, 0.3)
//                .setLeftIntakePower(-0.6)
//                .sleep(1000)
//                .setLeftIntakePower(0)
//                //.sleep(500)
//
//        ;
//        //putGroundPixel();
//        robot
//                .driveStraight(-6.0, 0, 0.3)
//                .turnToHeading(-90, 0.3)
////                .resetYaw()
//                .driveStraight(24, -90, 0.3)
//        ;
//        // FIXME:红色2号位置应该是apriltag 5
//        robot.initAprilTagVision();
//        robot.setManualExposure(4, 250);
//        AprilTagDetection desiredTag = null;
//        while (!robot.myOpMode.isStopRequested() && desiredTag == null) {
//            desiredTag = robot.getAprilTag(2);
//        }
//        robot.translateToAprilTag(
//                desiredTag,
//                14,
//                -2,
//                -90
//        );
//        robot.closeVision();
//        AutoDriveToAprilTag.goToAprilTag(robot, 14, 2,1);
//        robot.turnToHeading(-90, 0.3);
//        PutThePixel.putThePixel(robot);
//        robot
//                .sleep(2000)
//                .driveStraight(-4, -90, 0.3)
//                .driveStrafe(30, -90, 0.3)
//                //向右平移
//                .driveStraight(12.0, -90, 0.3)
//                .turnToHeading(0,0.5)
//
//        ;
    }
}
