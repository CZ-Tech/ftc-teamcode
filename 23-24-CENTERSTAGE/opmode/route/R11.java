package org.firstinspires.ftc.teamcode.opmode.route;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.test.PutThePixel;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class R11 {

    public static void run(RobotHardware robot) {
        robot.resetYaw();
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
        robot.setLeftIntakePower(0);
 //       +0
        robot.driveStraight(12, 0, 0.75);
        robot.driveStraight(17,45, 0.75)
                .sleep(500)
                .setLeftIntakePower(-0.6)
        .s(10,0.5)
                .setLeftIntakePower(0);
        list+="\n放置紫像素："+runtime.seconds();
        robot.syncRun(()->{
//                    robot.s(5,0.5)
                    robot.setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD);
                });

                                    //通过AprilTag走到板前
                robot.turnToHeading(-90, 0.3);
                robot.rush(20,-90)
                .driveStraight(-3, -90, 0.5);
                robot.turnToHeading(-90,0.3);
// FIXME:红色1号位置应该是apriltag 6
        robot.driveToAprilTag(6,11,-17.5,-90);
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 3.5) {
//            robot.driveRobot(Range.clip(0.2*(d-3.5),0,0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
        list+="\n放置黄色像素："+runtime.seconds();
//        robot.stopMotor();

        robot.holdHeading(-90, 0.3,1)
        .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(1000);
//
        robot.syncRun(()->{
//            robot.s(12,0.5)
               robot.setIntakeArmPosition(IntakeArmPosition.BACK)
                       .sleep(1000);
        });

        //回后台//TODO:
        robot
                .driveStraight(-6, -90, 0.3)
                .driveStrafe(35.0, -90, 0.3)
                .turnToHeading(0,0.3)
                .driveStrafe(12.0, 0, 0.3)
                .holdHeading(0,0.3,2)
        ;
        //+2
//        robot
//                //上板
//                .setLeftIntakePower(0)
//                .rush(24, 0)
//                .setLeftIntakePower(0)
//                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
//                .turnToHeading(-90,0.5)
//                .rush(30,-90)
//                .driveToAprilTag(5,11.0,-11.5,-90);
//        //FIXME:红色2号位置应该是apriltag 5
//        robot
//                .setIntakeBackPosition(IntakeBackPosition.OUT)
//                .sleep(1000)
//                .setIntakeArmPosition(IntakeArmPosition.BACK)
//                .driveStrafe(15,-90,0.8)
//                .turnToHeading(-90,0.3);
//        //紫像素
//        robot
//                .setLeftIntakePower(0.1)
//                .rush(-56.5,-90)
//                .setLeftIntakePower(-0.5)
//                .rush(-33,-90)
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
//                .driveToAprilTag(4,10,4,-90);
//        //        //FIXME:红色1号位置应该是apriltag 4
//
//        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
//        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
//        robot.sleep(1500)
//                .setIntakeArmPosition(IntakeArmPosition.BACK)
//                .turnToHeading(0,0.7)
//                .holdHeading(0,0.3,1000);
//        ;
//        list+="\n停回后台："+runtime.seconds();
//        robot.myOpMode.telemetry.addData("R23",list);
//        robot.myOpMode.telemetry.update();
//        while (robot.myOpMode.opModeIsActive()){}
    }
}
