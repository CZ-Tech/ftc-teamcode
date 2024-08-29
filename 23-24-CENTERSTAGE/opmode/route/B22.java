package org.firstinspires.ftc.teamcode.opmode.route;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//@Config
public class B22 {
    public static double DISTANCE_X = -4.5;
    public static double DISTANCE_Y = 20.5;
    public static double BOARD_DISTANCE_X = -5.5;
    public static double BOARD_DISTANCE_Y = 10;

    public static void run(RobotHardware robot) {
        robot
                .rush(27,0)
                .setLeftIntakePower(-0.6)
                .s(3)
                .setLeftIntakePower(0)
                .sleep(500)
                .turnToHeading(-90, 0.5)
                .w(10,0.5)
                .driveStrafe(-5, -90, 0.5);
//        robot.initAprilTagVision();
//        robot.setManualExposure(4, 250);

//        while (!robot.myOpMode.isStopRequested() && desiredTag == null) {
//            desiredTag = robot.getAprilTag(8);
//        }
//        robot.translateToAprilTag(
//                desiredTag,
//                23.25,
//                -4.875,
//                -90
//        );
//        robot.turnToHeading(-90,0.5);
//        robot.setIntakeArmPosition(IntakeArmPosition.TOP);
//        robot.closeVision();
//        robot.turnToHeading(-90,0.5);
//        robot.setIntakeArmPosition(IntakeArmPosition.TOP);
//
//        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT1);
//        robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
//        double st =0;//单位s,提供50hz的舵机基准信号
//        while(st<1){
//            robot.setIntakeRoller(IntakeRollerPosition.STOP);
//            robot.sleep(19);
//            st=st+0.02;
//        }//电调启动时油门要在中位
//        st=0;
//        while(st<1.5){
//            robot.setIntakeRoller(IntakeRollerPosition.IN);
//            robot.sleep(19);
//            st=st+0.02;
//        }
//        robot.setIntakeFrontPosition(IntakeFrontPosition.IN);
//        robot.setIntakeArmPosition(IntakeArmPosition.BACK);
//        st=0;
//        while(st<0.1){
//            robot.setIntakeRoller(IntakeRollerPosition.STOP);
//            robot.sleep(19);
//            st=st+0.02;
//        }
        robot.turnToHeading(-90,0.5);
        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT1);
        //已修改为正确的编号
        //TODO：未调试
        robot.driveToAprilTag(9,20.5,-3.5,-90);
//        robot.driveToAprilTag(9,DISTANCE_Y,DISTANCE_X,-90);
//        robot.closeVision();
//        // April tag 图像识别结束
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
            robot.setIntakeRoller(IntakeRollerPosition.IN);
            robot.sleep(19);
            st = st + 0.02;
        }

//        robot.s(2);

        st = 0;
        while (st < 0.1) {
            robot.setIntakeRoller(IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }
        robot.setIntakeFrontPosition(IntakeFrontPosition.IN);
        robot.setIntakeArmPosition(IntakeArmPosition.LIFTALITTLE);
        robot.sleep(300);
        robot.driveStrafe(-22, -80, 0.8)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
                .turnToHeading(90, 0.8)
            .rush(75, 90)
            .driveStrafe(-28, 90, 0.8)
            .turnToHeading(90, 0.5)
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD);

//            .initAprilTagVision()
//            .setManualExposure(5, 250);
        robot.holdHeading(90,0.3,1);
        robot.driveToAprilTag(2, 10, -4, 90);
        robot.closeVision();
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 3.75) {
//            robot.driveRobot(Range.clip(0.2 * (d - 3.75), 0, 0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);}
//        robot.closeVision();
//        AutoDriveToAprilTag.goToAprilTag(robot, 14, 2,1);
        robot
                .holdHeading(90, 0.3,1)

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
                .turnToHeading(0,0.5)
                .holdHeading(0,0.3,1000)
        ;
    }
}
