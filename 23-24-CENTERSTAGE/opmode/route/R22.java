package org.firstinspires.ftc.teamcode.opmode.route;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//@Config
public class R22 {
    public static double DISTANCE_X = -4;
    public static double DISTANCE_Y = 8.5;
    public static double BOARD_DISTANCE_X = -6;
    public static double BOARD_DISTANCE_Y = 10;

    public static void run(RobotHardware robot) {
        // FIXME:红色2号位置应该是apriltag 5
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
        robot.setIntakeBackPosition(RobotConstants.IntakeBackPosition.IN);
        robot
                //向前直走
                .w(27.0, 0.8)//TODO:待调试
                //启动飞轮放像素
                .setLeftIntakePower(-0.6)
                .s(2)
                .setLeftIntakePower(0)
                .sleep(200)
                ;
        list+="\n放置紫像素："+runtime.seconds();
        robot
                //向后直走回到初始位置
                .s(4.0)
                //向右平移至后台
                .G(90)
        ;

//        robot.initAprilTagVision();
        robot.setManualExposure(4, 250);
        robot.syncRun(
            () -> {
                robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);
                robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
            }

        );
//        thread.start();

        robot.turnToHeading(90,0.3);
        //已修改为正确的编号
//        telemetry.addData("1", null);   //TODO:调试BUG专用
//        telemetry.update();
        robot.driveToAprilTag(8,21,-4,90);
//        telemetry.addData("4", null);   //TODO:调试BUG专用
//        telemetry.update();
        robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_EAT1);
        robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
        double st =0;//单位s,提供50hz的舵机基准信号
        while(st<1){
            robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.STOP);
            robot.sleep(19);
            st=st+0.02;
        }//电调启动时油门要在中位
        robot.driveStraight(2, 90, 0.3);
        st=0;
        while(st<0.8){
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
                .turnToHeading(-90,0.5)
                .s(6)
                .a(32.0, 0.5)
                .rush(80.0, -90)
                .d(25.0, 0.5)
                .turnToHeading(-90,0.3)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);

        //FIXME:红色2号位置应该是apriltag 5
    //robot.driveToAprilTag(2,14,-4.5,-90);//放左边
        robot.driveToAprilTag(5,10,-4.5,-90);//放右边
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 4) {
//            robot.driveRobot(Range.clip(0.2 * (d - 4), 0, 0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
        list+="\n放置黄色像素："+runtime.seconds();
        robot
                .sleep(500)
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.TOP)
                .sleep(500)
               // .driveStrafe(6.5, -90, 0.5)//放左边
                .driveStrafe(5.5, -90, 0.5)//放右边
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
                .holdHeading(0, 0.3, 1);
        list+="\n放完像素："+runtime.seconds();

        robot.myOpMode.telemetry.addData("R22",list);
        robot.myOpMode.telemetry.update();
        while (robot.myOpMode.opModeIsActive()){}

    }
}
