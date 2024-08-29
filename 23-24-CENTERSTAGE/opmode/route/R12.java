package org.firstinspires.ftc.teamcode.opmode.route;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.test.PutThePixel;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//@Config
public class R12 {
    //    @Override
    public static void run(RobotHardware robot) {

        robot.resetYaw();
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
// +2
 /*

        robot
                //紫像素
                .rush(25.5, 0)
                .setLeftIntakePower(-0.8)
                .driveStraight(-1,0,0.5)
                .setLeftIntakePower(0)
                //上板
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .turnToHeading(-90,0.7)
                .rush(30,-90)
                .driveToAprilTag(5,11.5,-4,-90);
        //FIXME:红色2号位置应该是apriltag 5
        robot
                .setIntakeBackPosition(IntakeBackPosition.OUT)
                .sleep(1000)
                        .setIntakeArmPosition(IntakeArmPosition.BACK)
                .sleep(1000)
                        .driveStrafe(5,-90,0.8)
                        .turnToHeading(-90,0.3)
                        .rush(-90,-88)
                        .turnToHeading(90,1)
        .turnToHeading(89,0.3);
//        robot.driveToAprilTag(10,18.75,3,90);
        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT1);
        robot.driveToAprilTag(8,19,-4.5,90);

        double st = 0;//单位s,提供50hz的舵机基准信号
        while (st < 0.5) {
            robot.setIntakeRoller(IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }//启动时信号要在中位
        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
        robot.setIntakeBackPosition(IntakeBackPosition.IN);
        st = 0;
        while (st < 0.5) {
            robot.setIntakeRoller(IntakeRollerPosition.IN);
            robot.sleep(19);
            st = st + 0.02;}

        robot.s(1);
        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT2);
        robot.w(1.2);

        st = 0;
        while (st < 1) {
            robot.setIntakeRoller(IntakeRollerPosition.SLOWIN);
            robot.sleep(19);
            st = st + 0.02;}
        st = 0;
        while (st < 0.1) {
            robot.setIntakeRoller(IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }
        robot.setIntakeBackPosition(IntakeBackPosition.IN);
        robot.setIntakeFrontPosition(IntakeFrontPosition.IN);

        robot.setIntakeArmPosition(IntakeArmPosition.LIFTALITTLE)
                .turnToHeading(-90,0.8)
                .driveStraight(-7, -90, 0.8)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
                .driveStrafe(-33, -90, 0.8)
                .turnToHeading(-90,0.3)
                .rush(80, -90)
                .driveStrafe(19, -90, 0.8)
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .turnToHeading(-90,0.3)
                .driveToAprilTag(4,10.5,-4,-90);
        //        //FIXME:红色1号位置应该是apriltag 4

        robot.setIntakeBackPosition(IntakeBackPosition.OUT);
        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
        robot.sleep(1200)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
                .turnToHeading(0,0.7)
                .holdHeading(0,0.3,1000);
         */


        //+0 NEW
        robot
                //Purple Pixel
                .rush(26, 0)
                .setLeftIntakePower(-0.6)
                .driveStraight(-3, 0, 0.5)
                .setLeftIntakePower(0);
        list+="\n放置紫像素："+runtime.seconds();

        //OnBoard
        robot
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .turnToHeading(-90, 0.7)
                .holdHeading(-90, 0.3, 1)
                .driveStraight(28, -90, 0.7)
                .driveToAprilTag(5, 11.5, -5.5, -90, 0.3);
//        robot.driveRobot(0.3, 0, 0);
        //FIXME:红色2号位置应该是apriltag 5

//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 2.5) {
//            robot.driveRobot(Range.clip(0.2*(d-3),0,0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
//        robot.stopMotor();
        list+="\n放置黄色像素："+runtime.seconds();

        robot
                .holdHeading(-90, 0.3, 1)
                .setIntakeBackPosition(IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
                .sleep(1000)
                .driveStrafe(30, -90, 0.5)
                .turnToHeading(0, 0.5)
                .holdHeading(0, 0.3, 1);
        list+="\n停回后台："+runtime.seconds();
        robot.myOpMode.telemetry.addData("R12",list);
        robot.myOpMode.telemetry.update();
        while (robot.myOpMode.opModeIsActive()){}
    }
}
