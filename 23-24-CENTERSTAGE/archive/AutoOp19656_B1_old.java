package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Deprecated
@Autonomous(name = "自动操控模式_B1_Test", group = "Robot", preselectTeleOp = "手动阶段")
@Disabled
public class AutoOp19656_B1_old extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();
//    Recognition recognition=null;
    public void putGroundPixel(){
        robot
                //启动飞轮放像素
                .setLeftIntakePower(-1.0)
                //等待0.5s
                .sleep(500)
                //关闭飞轮
                .setLeftIntakePower(0)
        ;
    }

    @Override
    public void runOpMode() {
        // region 程序初始化阶段
        robot.init();
        robot.launchDroneServo.setPosition(1.0);
        robot.initDoubleVision();
        telemetry.addData("Status", "Initialized");

        waitForStart();
        robot.resetYaw();
        runtime.reset();
        robot.G(-83);

        // mission 0=未检测 1-左 2-中 3-右
        int MISSION = 0;
        // 比赛开始阶段进行图像检测（可能会出错，有问题的话就放waitForStart下面再试试。
        while (MISSION == 0) {
            //  blue-beacon
            //  red-beacon
            //  white-pixel
            Recognition recognition = robot.getTfod("blue-beacon");
            if (recognition != null) {
                telemetry.addData("left", "%4.0f", recognition.getLeft());
                if (recognition.getLeft() < 150) {
                    MISSION = 1;
                } else if (recognition.getLeft() > 400) {
                    MISSION = 3;
                } else {
                    MISSION = 2;
                }
            }

            robot.driveStrafe(2, -90, 0.5);
            robot.driveStrafe(-2, -90, 0.5);
            sleep(1000);
            telemetry.addData("MISSION", "%4d", MISSION);
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.update();
            if(runtime.seconds()>10)MISSION=2;

        }

        // endregion

        // Wait for the game to start (driver presses PLAY)



//        MISSION = 2;
        switch (MISSION) {
            case 1 :
                robot
                    .driveStraight(22.0, -90, 0.5)//TODO:待调试
                        //向右
                    .driveStrafe(-12.0, -90, 0.5)
                        //启动飞轮放像素
                    .setLeftIntakePower(-0.5)
                    .sleep(500)
                    .setLeftIntakePower(0)
                        //边转边走
                    .G(0.0)
                    .G(90)
                    .driveStrafe(40.0, 90, 0.5);

                robot
                        //像素上板
                    .setOneTimeMotorPower(0.5)
                    .sleep(500)
                    .setOneTimeMotorPower(0.0)
                    .sleep(1000)
                    .driveStrafe(-1.0)
                    .setOneTimeMotorPower(-0.5)
                    .sleep(500)
                    .setOneTimeMotorPower(0.0)
                        //停回后台
                    .driveStraight(18.0, 90, 0.5)
                    .driveStrafe(12.0, 90, 0.5)
                ;break;
            case 2 :
                robot
                    .driveStraight(30.0, -90, 0.5)//TODO:待调试
                    .driveStraight(-3, 90, 0.5)
                        //启动飞轮放像素
                    .setLeftIntakePower(-0.5)
                    .sleep(1000)
                    .setLeftIntakePower(0)
                        //边转边走
                    .G(0)
                    .G(90)
                    .driveStrafe(55.0, 90, 0.5)
                    .driveStraight(-9.0, -90, 0.5)
                        //像素上板
                    .setOneTimeMotorPower(0.5)
                    .sleep(500)
                    .setOneTimeMotorPower(0.0)
                    .sleep(1500)
                    .driveStrafe(-1.0, 90, 0.5)
                    .setOneTimeMotorPower(-0.5)
                    .sleep(500)
                    .setOneTimeMotorPower(0.0)
                        //停回后台
                    .driveStraight(24.0, 90, 0.5)
                    .driveStrafe(12.0, 90, 0.5)
                ;break;
            case 3 :
                robot
                        //向前直走
                    .driveStraight(26.0, -90, 0.5)//TODO:待调试
                    .driveStrafe(18.0, -90, 0.5)
                        //启动飞轮放像素
                    .setLeftIntakePower(-1.0)
                    .sleep(500)
                    .setLeftIntakePower(0)
                        //边走边转
                        .driveStrafe(66.0, -90, 0.5)
                        .G(0)
                        .G(90)
                        .driveStraight(-8.0, -90, 0.5)

                        //像素上板
                    .setOneTimeMotorPower(0.5)
                    .sleep(500)
                    .setOneTimeMotorPower(0.0)
                    .sleep(1500)
                    .driveStrafe(-1.0)
                    .setOneTimeMotorPower(-0.5)
                    .sleep(500)
                    .setOneTimeMotorPower(0.0)
                        //停回后台
                    .driveStraight(30.0, 90, 0.5)
                    .driveStrafe(12.0, 90, 0.5)
                ;break;
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        robot.closeVision();
    }
}
