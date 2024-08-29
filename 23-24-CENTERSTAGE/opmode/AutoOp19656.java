package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.Alliance;
import org.firstinspires.ftc.teamcode.vision.DetectionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.LoadProperties;

import java.util.ArrayList;

public class AutoOp19656 extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    DetectionProcessor openCV = new DetectionProcessor(telemetry);
    AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
    LoadProperties args_B13 = new LoadProperties("/sdcard/FIRST/tflitemodels/args.properties", "B13");

    int mission = 0;
    ArrayList args;

    @Override


    public void runOpMode() {
        // region 程序初始化阶段
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//FTCDashBoard专属Telemetry
        robot.init();
        robot.initVision(openCV);
        robot.setCameraMode(ExposureControl.Mode.AperturePriority);

        robot.resetYaw();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            //args = args_B13.load();
            telemetry.addData("Succeed in loading properties!", null);
        } catch (Throwable e) {
            throw new RuntimeException(e);
        }
        // mission 0=未检测 1-左 2-中 3-右
        while (opModeInInit()) {
            mission = openCV.getResult();
            telemetry.addData("Status", "Initialized");
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.addData("Mission:", mission);
            telemetry.update();
        }

        waitForStart();
        robot.closeVision();
        robot.initAprilTagVision();
        robot.setManualExposure(4, 250);
        //红心特供
//        robot.sleep(11500);
//        robot.setDoorOpener(true);
        // endregion
    }

//    @Autonomous(name = "AutoR1🔴🎬", group = "A")
    @Autonomous(name = "AutoR1🔴🎬", group = "A", preselectTeleOp = "Tele🔴")
    public static class AutoR1 extends AutoOp19656 {
        @Override
        public void runOpMode() {
            openCV.setTeamColor(Alliance.RED.getColor());
            super.runOpMode();
            if (mission == 1) robot.route.r11();
            if (mission == 2) robot.route.r12();
            if (mission == 3) robot.route.r13();
        }
    }

//    @Autonomous(name = "AutoR2🔴🎅", group = "A")
    @Autonomous(name = "AutoR2🔴🎅", group = "A", preselectTeleOp = "Tele🔴")
    public static class AutoR2 extends AutoOp19656 {
        @Override
        public void runOpMode() {
            openCV.setTeamColor(Alliance.RED.getColor());
            super.runOpMode();
            if (mission == 1) robot.route.r21();
            if (mission == 2) robot.route.r22();
            if (mission == 3) robot.route.r23();
        }
    }

//    @Autonomous(name = "AutoB1🔵🎬", group = "A")
    @Autonomous(name = "AutoB1🔵🎬", group = "A", preselectTeleOp = "Tele🔵")
    public static class AutoB1 extends AutoOp19656 {
        @Override
        public void runOpMode() {
            openCV.setTeamColor(Alliance.BLUE.getColor());
            super.runOpMode();
            if (mission == 1) robot.route.b11();
            if (mission == 2) robot.route.b12();
            if (mission == 3) {
                robot.route.b13(args);
            }
        }
    }

//    @Autonomous(name = "AutoB2🔵🎅", group = "A")
    @Autonomous(name = "AutoB2🔵🎅", group = "A", preselectTeleOp = "Tele🔵")
    public static class AutoB2 extends AutoOp19656 {
        @Override
        public void runOpMode() {
            openCV.setTeamColor(Alliance.BLUE.getColor());
            super.runOpMode();
            if (mission == 1) robot.route.b21();
            if (mission == 2) robot.route.b22();
            if (mission == 3) robot.route.b23();
        }
    }
}
