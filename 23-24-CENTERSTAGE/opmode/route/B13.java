package org.firstinspires.ftc.teamcode.opmode.route;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.test.PutThePixel;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class B13 {
    public static void run(RobotHardware robot) {
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
        robot.driveStraight(15, 0, 0.5);
        robot.turnToHeading(-45);
        robot.driveStraight(7.5, -45, 0.5);
        //启动飞轮放像素
        robot.setLeftIntakePower(-0.6);
        robot.driveStraight(-3, -45, 0.5)
                .setLeftIntakePower(0)
                .sleep(500);
        list+="\n放置紫像素："+runtime.seconds();
        //TODO:待调试
        //倒出来
        robot.driveStrafe(-18, -45, 0.5);
        robot.turnToHeading(90, 0.3);
        robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);
        robot.driveStraight(20, 90, 0.5)

        //robot
        //.sleep(10000)

//                        //向前直走

        //.driveStraight(0.3, 26, 0)
        //.driveStrafe(0.3, 15.5, 0)


//                        边走边转
        //.driveStrafe(0.3, -30.0, 0)
        //.turnToHeading(0.3, 90)
        //.driveStraight(0.3, -8.0, 90)
        //.driveStrafe(0.3, 34.0, 90)
        ;

//        robot.initAprilTagVision();
//        robot.setManualExposure(4, 250);
        robot.holdHeading(90,0.3,1);

        robot.driveToAprilTag(3,10.5,-3,90);
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 3.75) {
//            robot.driveRobot(Range.clip(0.2*(d-3.75),0,0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
        robot.stopMotor();
        robot.closeVision();
        list+="\n放置黄色像素："+runtime.seconds();
        //.d(1.0)
        //向后直走
        robot
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(1000)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.BACK)
                .sleep(1000)
        //原回后台程序
                .driveStraight(-4, 90, 0.3)
                .driveStrafe(-30, 90, 0.3)
                .driveStraight(14.0, 90, 0.3)
                .turnToHeading(0,0.5)
                .driveStrafe(-12,90,0.3)
                .holdHeading(0,0.3,1)
        ;
        list+="\n停回后台："+runtime.seconds();
        robot.myOpMode.telemetry.addData("B13",list);
        robot.myOpMode.telemetry.update();
        while (robot.myOpMode.opModeIsActive()){}
//                    goBackStage(-24.0);

    }
}
