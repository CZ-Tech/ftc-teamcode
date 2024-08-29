package org.firstinspires.ftc.teamcode.opmode.route;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.test.PutThePixel;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class R13 {
    public static void run(RobotHardware robot) {
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
//  +2
//        robot
//                //上板
//                .setLeftIntakePower(0)
//                .driveStraight(5, 0, 0.5)
//                .turnToHeading(-90,0.5)
//                .rush(24, -90)
//                .driveStrafe(-20,-90,0.5)
//                .turnToHeading(-90,0.5)
//                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
//                .driveToAprilTag(6,11.0,-3,-90);
//        //FIXME:红色2号位置应该是apriltag 6
//        robot
//                .setIntakeBackPosition(IntakeBackPosition.OUT)
//                .sleep(1000)
//                .setIntakeArmPosition(IntakeArmPosition.BACK)
//                .driveStrafe(-2, -90, 0.5)
//                .turnToHeading(-90,0.3);
//        //紫像素
//        robot
//                .setLeftIntakePower(0.1)
//                .rush(-34,-90)
//                .setLeftIntakePower(-0.5)
//                .rush(-55,-90)
//                .setLeftIntakePower(-0)
//                .turnToHeading(90,1)
//                .turnToHeading(90,0.3);
////        robot.driveToAprilTag(10,18.75,3,90);
//        robot.driveToAprilTag(8,21,-4.5,90);
//        robot.turnToHeading(90,0.3);
//        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT1);
//        robot.driveStraight(2, 90, 0.3);
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
//        while (st < 0.4) {
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
//        robot.setIntakeArmPosition(IntakeArmPosition. BACK);
//        robot
//                .driveStrafe(33, 90, 0.8)
//                .turnToHeading(-90,0.8)
//                .rush(80, -90)
//                .driveStrafe(19, -90, 0.8)
//                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
//                .turnToHeading(-90,0.5)
//                .driveToAprilTag(4,10,0,-90);
//        //        //FIXME:红色1号位置应该是apriltag 4
//
//        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
//        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
//        robot.sleep(1500)
//                .setIntakeArmPosition(IntakeArmPosition.BACK)
//                .turnToHeading(0,0.7)
//                .holdHeading(0,0.3,1000);
        //+0
        robot
                //向右平移
                .driveStraight(9.0, 0, 0.5)
                //向前直走
                .driveStraight(22.0, -45, 0.5)
                .setLeftIntakePower(-0.6)
                .driveStraight(-5.0, -45, 0.5)
                .setLeftIntakePower(0)
                .sleep(1000)

        ;
        list+="\n放置紫像素："+runtime.seconds();

//        放紫像素
        robot

                //向右平移
                .turnToHeading(-90, 0.5)
                .driveStraight(24, -90, 0.5)//TODO:待调试
                .driveStrafe(-8, -90, 0.5);
        //上板
        // FIXME:红色3号位置应该是AprilTag 6

        robot.driveToAprilTag(6, 10, -2, -90, 0.5);
        robot.myOpMode.telemetry.addData("x","drivetoapril");
        robot.myOpMode.telemetry.update();
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 4) {
//            robot.driveRobot(Range.clip(0.2 * (d - 4), 0, 0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
//        robot.stopMotor();
//        robot.myOpMode.telemetry.addData("x","距离传感器结束");
        robot.myOpMode.telemetry.update();

        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .sleep(1500)
                .setIntakeBackPosition(IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.BACK);
        list+="\n放置黄色像素："+runtime.seconds();

        robot
                .sleep(1000)
                .driveStraight(-4, -90, 0.5)//TODO:待调试
                .driveStrafe(20, -90, 0.5)
                //向右平移
                .turnToHeading(0, 0.5)
                .driveStrafe(12.0, 0, 0.5)
                .holdHeading(0, 0.5, 2)

        ;
        list+="\n停回后台："+runtime.seconds();
        robot.myOpMode.telemetry.addData("R13",list);
        robot.myOpMode.telemetry.update();
        while (robot.myOpMode.opModeIsActive()){}

    }
}
