package org.firstinspires.ftc.teamcode.opmode.route;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.test.PutThePixel;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.IntakeArmPosition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//@Config
public class R21 {
    public static double DISTANCE_X = 20.5;
    public static double DISTANCE_Y = 20.5;
    public static double BOARD_DISTANCE_X = -3;
    public static double BOARD_DISTANCE_Y = 10;
    public static void run(RobotHardware robot) {
        // FIXME:红色1号位置应该是AprilTag 4
        //向前直走
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
        robot.driveSpeed = 0.3;
        robot.resetYaw();
        robot.driveStraight(12, 0, 0.75);
        robot.driveStraight(16,45, 0.75);

//                        //启动飞轮放像素
        robot.setLeftIntakePower(-0.6)
                .sleep(500)
                .setLeftIntakePower(0)
                .sleep(400);
        list+="\n放置紫像素："+runtime.seconds();
        //向后直走回到初始位置
        //TODO:待调试
        robot.s(3)
                //向前走到中门
                .turnToHeading(90.0, 0.5)
                .s(2)
                .driveStrafe(9, 90, 0.7);


// FIXME:红色1号位置应该是AprilTag 4

//        robot.initAprilTagVision();
//        robot.setManualExposure(4, 250);
      robot.holdHeading(90, 0.3, 0.75);
        robot.setIntakeArmPosition(IntakeArmPosition.TOP);
        //已修改为正确的编号
        robot.driveToAprilTag(8, 21, 21, 90);
//        robot.closeVision();
//        // April tag 图像识别结束
        robot.holdHeading(90, 0.3, 0.75);
        robot.setIntakeArmPosition(IntakeArmPosition.AUTO_EAT1);
        robot.turnToHeading(90, 0.5);
        double st = 0;//单位s,提供50hz的舵机基准信号
        while (st < 0.5) {
            robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }//启动时信号要在中位
        robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
        robot.driveStraight(2, 90, 0.5);
        st = 0;
        while (st < 0.8) {
            robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.IN);
            robot.sleep(19);
            st = st + 0.02;
        }
        robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.IN);
        robot.s(5.5);
        robot.setIntakeArmPosition(IntakeArmPosition.BACK);
        st = 0;
        while (st < 0.1) {
            robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.STOP);
            robot.sleep(19);
            st = st + 0.02;
        }
        list+="\n拾取堆放区像素："+runtime.seconds();
        robot
//                .turnToHeading(0)
//                .rush(25,0)
                .driveStrafe(10, 90, 0.8)
                .turnToHeading(-90, 0.8)
                .rush(74, -90)
                .driveStrafe(17, -90, 0.8)
                .turnToHeading(-90, 0.5)
                .holdHeading(-90, 0.3, 1)
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD);
//                .initAprilTagVision()
//                .setManualExposure(5, 250);

        robot.driveToAprilTag(4, BOARD_DISTANCE_Y, BOARD_DISTANCE_X, -90);
        robot.closeVision();
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 3.75) {
//            robot.driveRobot(Range.clip(0.2 * (d - 3.75), 0, 0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
        list+="\n放置黄色像素："+runtime.seconds();
        robot
                .holdHeading(-90, 0.3, 1)
                .sleep(500)
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.TOP)
                .sleep(500)
                .driveStrafe(9, -90, 0.5)
                .setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD)
                .sleep(1000)
                .setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT)
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.IN)
                .sleep(500)
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(IntakeArmPosition.BACK)
                .sleep(500);
        list+="\n放置白色像素："+runtime.seconds();
        robot
                .turnToHeading(0, 0.7)
                .holdHeading(0, 0.3, 1)
        ;
        list+="\n放完像素："+runtime.seconds();
        robot.myOpMode.telemetry.addData("R21",list);
        robot.myOpMode.telemetry.update();
        while (robot.myOpMode.opModeIsActive()){}
    }
}
