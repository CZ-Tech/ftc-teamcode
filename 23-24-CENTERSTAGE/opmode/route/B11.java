package org.firstinspires.ftc.teamcode.opmode.route;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.test.PutThePixel;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class B11 {
    public static void run(RobotHardware robot) {
        ElapsedTime runtime = new ElapsedTime();
        String list="";
        runtime.reset();
        robot
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.BACK)
                .driveStraight(20, 0, 0.5)
                .turnToHeading(45, 0.5)
                .driveStraight(4, 45, 0.5)

                .setLeftIntakePower(-0.6)
                .driveStraight(-7, 45, 0.5)
                .setLeftIntakePower(0);
        list+="\n放置紫像素："+runtime.seconds();
        robot        //通过AprilTag走到板前
                .turnToHeading(90, 0.5)
                .a(2)
                .driveStraight(25.0, 90, 0.5);

//        robot.initAprilTagVision();
//        robot.setManualExposure(4, 250);
        robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);
//        AprilTagDetection desiredTag = null;
//        while (!robot.myOpMode.isStopRequested() && desiredTag == null) {
//            desiredTag = robot.getAprilTag(1);
//        }
//        robot.translateToAprilTag(
//                desiredTag,
//                12,
//                -6.75,
//                90
//        );
        robot.driveToAprilTag(1,10.75,-5.5,90);
//        double d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        while (d > 3.5) {
//            robot.driveRobot(Range.clip(0.2*(d-3.5),0,0.3), 0, 0);
//            d = robot.sensorDistance.getDistance(DistanceUnit.INCH);
//        }
        robot.closeVision();
//        AutoDriveToAprilTag.goToAprilTag(robot, 16, 1,-1);

        robot.holdHeading(90,0.5,1);
//        PutThePixel.putThePixel(robot);
        list+="\n放置黄色像素："+runtime.seconds();

        robot
                .setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT)
                .sleep(2000)
                .setIntakeArmPosition(RobotConstants.IntakeArmPosition.BACK)
                .sleep(1000)
//原回后台程序
                .driveStraight(-4.0, 90, 0.5)
                .driveStrafe(-14.0, 90, 0.5)
                .driveStraight(15.0, 90, 0.5)
                .turnToHeading(0,0.5)
                .driveStrafe(-6,90,0.5)
                .holdHeading(0,0.3,1)
        ;
        list+="\n停回后台："+runtime.seconds();
        robot.myOpMode.telemetry.addData("R23",list);
        robot.myOpMode.telemetry.update();
        while (robot.myOpMode.opModeIsActive()){}


//

//                .driveStrafe(0.3,24.0,90)
//                .driveStraight(0.3,12.0,90)

        ;
    }
}
