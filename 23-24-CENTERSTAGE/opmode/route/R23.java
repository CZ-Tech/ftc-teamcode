package org.firstinspires.ftc.teamcode.opmode.route;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
//@Config
public class R23 {
    public static double DISTANCE_X = 6;
    public static double DISTANCE_Y = 21;


    public static void run(RobotHardware robot) {
        // FIXME:红色3号位置应该是apriltag 6
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
        robot
                //向前直走
                .driveStraight(12, 0, 0.5)
                //向右至像素
                .driveStraight(22.5, -45, 0.5)
                //启动飞轮放像素
                .setLeftIntakePower(-0.6)
                .s(2)
                .setLeftIntakePower(0)
                .sleep(500);
        list+="\n放置紫像素："+runtime.seconds();
        robot
                .driveStraight(-8, -45, 0.5)
                .turnToHeading(90, 0.5);
        robot.syncRun(
                () -> {
                    robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);
                    robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
                }

        )
                .driveStrafe(19, 90, 0.5);
        //已修改为正确的编号 right+ left-
        robot.driveToAprilTag(8,DISTANCE_Y,DISTANCE_X,90);
        robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_EAT1);
        robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
        double st =0;//单位s,提供50hz的舵机基准信号
        while(st<1){
            robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.STOP);
            robot.sleep(19);
            st=st+0.02;
        }//启动时信号要在中位
        robot.driveStraight(3, 90, 0.3);
        st=0;
        while(st<0.6){
            robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.IN);
            robot.sleep(19);
            st=st+0.02;
        }
        robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.IN);
        robot.driveStraight(-2, 90, 0.3);
        robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.BACK);
        st=0;
        while(st<0.1){
            robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.STOP);
            robot.sleep(19);
            st=st+0.02;
        }
        robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.BACK);
        robot.sleep(500);
        list+="\n拾取堆放区像素："+runtime.seconds();
        robot
                .turnToHeading(-90, 0.5)
                .a(20.0, 0.5)
                .rush(85.0, -90)
                .d(32.0, 0.5)
                .turnToHeading(-90,0.5)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);
            robot.driveToAprilTag(6,10.5,-4.5,-90);
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 3.75) {
//            robot.driveRobot(Range.clip(0.2 * (d - 3.75), 0, 0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
        list+="\n放置黄色像素："+runtime.seconds();
        robot
                .turnToHeading(-90, 0.5)
                .sleep(500)
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.TOP)
                .sleep(500)
                .driveStrafe(-8, -90, 0.5)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD)
                .sleep(1000)
                .setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT)
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.IN)
                .sleep(500)
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.BACK)
                .sleep(500);
        list+="\n放置白色像素："+runtime.seconds();
        robot
                .turnToHeading(0, 0.7)
                .holdHeading(0, 0.3, 1)
        ;
        list+="\n放完像素："+runtime.seconds();
        robot.myOpMode.telemetry.addData("R23",list);
        robot.myOpMode.telemetry.update();

        while (robot.myOpMode.opModeIsActive()){}
    }
}
