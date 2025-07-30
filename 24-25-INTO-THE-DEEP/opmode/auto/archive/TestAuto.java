package org.firstinspires.ftc.teamcode.opmode.auto.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;

//@Config
@Disabled
@Autonomous(name = "TestAuto", group = "Auto")
public class TestAuto extends LinearOpMode {
    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    public static  int SLEEPTIME = 245;
    public static  double DRIVESPEED = 0.3;//驾驶速度
    public static  double SPEDIS = 50;//推动Sample的距离

    @Override
    public void runOpMode() {

        // region Initialization
        runtime.reset();
        robot.init(this);
//        robot.imu.initialize(new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                )
//        ));

        sleep(1000);
        robot.drivetrain.resetYaw();
        telemetry.addData(">", "Hardware Initialized");

        while (opModeInInit()){
            robot.drivetrain.getSteeringCorrection(0,0.03);

        }
        // Wait for the 'Start' button pressed
        waitForStart();
        // endregion
        // Main Method

        robot.drivetrain
                .driveStraight(24.0, 0, DRIVESPEED)
                .sleep(300)//第一个目标
        ;


        while (opModeIsActive()) {
            robot.drivetrain.getHeading();
            telemetry.update();
        }

    }

    // region 统一入口
    //Red alliance

//    @Autonomous(name = "Auto🔴", group = "Auto", preselectTeleOp = "Duo🔴")
    public class AutoRed extends AutoOrigin {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    //Blue alliance

//    @Autonomous(name = "Auto🔵", group = "Auto", preselectTeleOp = "Duo🔵")
    public class AutoBlue extends AutoOrigin {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            super.runOpMode();
        }
    }
    // endregion
}
