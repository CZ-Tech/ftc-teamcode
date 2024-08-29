package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.Alliance;
import org.firstinspires.ftc.teamcode.vision.DetectionProcessor;

@Deprecated
@Autonomous(name = "🔴自动操控模式_R2", group = "Robot", preselectTeleOp = "手动阶段")
@Disabled
public class AutoOp19656_R2 extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    DetectionProcessor processor = new DetectionProcessor(telemetry);

    @Override
    public void runOpMode() {

        // region 程序初始化阶段
        robot.init();
        robot.vision.init(processor);
        robot.vision.setCameraMode(ExposureControl.Mode.AperturePriority);
        processor.setTeamColor(Alliance.RED.getColor());
        robot.resetYaw();

        // mission 0=未检测 1-左 2-中 3-右
        int mission = 0;
        while (opModeInInit()) {
            mission = processor.getResult();
            telemetry.addData("Status", "Initialized");
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.addData("Mission:", mission);
            telemetry.update();
        }
        robot.vision.closeVision();
        // endregion

//        robot.setIntakeArmPosition(0);

        if (mission == 1) robot.route.r21();
        if (mission == 2) robot.route.r22();
        if (mission == 3) robot.route.r23();
    }
}


// region 老代码

/*
    public void runOpMode() {
        // region 程序初始化阶段
        robot.init();
        robot.launchDroneServo.setPosition(1.0);
        robot.initDoubleVision();
        telemetry.addData("Status", "Initialized");

        // mission 0=未检测 1-左 2-中 3-右
        int MISSION = 2;
        // 比赛开始阶段进行图像检测（可能会出错，有问题的话就放waitForStart下面再试试。
        while (opModeInInit()) {
            //  blue-beacon
            //  red-beacon
            //  white-pixel
            Recognition recognition = robot.getTfod("rb");
            if (recognition != null) {
                telemetry.addData("left", "%4.0f", recognition.getLeft());
                if (recognition.getLeft() < 150) {
                    MISSION = 1;
                } else if (recognition.getLeft() > 400) {
                    MISSION = 3;
                } else{
                    MISSION = 2;
                }
            }
            telemetry.addData("MISSION", "%4d", MISSION);
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.update();
        }
        // endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.resetYaw();



        telemetry.addData("Path", "Complete");
        telemetry.update();

        robot.closeVision();
    }
 */
// endregion
