package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.Alliance;
import org.firstinspires.ftc.teamcode.vision.DetectionProcessor;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;
@Deprecated
@Autonomous(name = "🔵自动操控模式_B2", group = "Robot", preselectTeleOp = "手动阶段")
@Disabled
public class AutoOp19656_B2 extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    DetectionProcessor processor = new DetectionProcessor(telemetry);

    @Override
    public void runOpMode() {

        // region 程序初始化阶段
        robot.init();
        robot.vision.init(processor);
        robot.vision.setCameraMode(ExposureControl.Mode.AperturePriority);
        processor.setTeamColor(Alliance.BLUE.getColor());
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

        robot.setIntakeArmPosition(IntakeArmPosition.BACK);

        if (mission == 1) robot.route.b21();
        if (mission == 2) robot.route.b22();
        if (mission == 3) robot.route.b23();
    }
}

// region 老程序初始化阶段

/*

        robot.init();
        robot.launchDroneServo.setPosition(1.0);
        robot.initDoubleVision();
        telemetry.addData("Status", "Initialized");
        waitForStart();
        robot.resetYaw();
        runtime.reset();
        robot.G(-89);

        // mission 0=未检测 1-左 2-中 3-右
        int MISSION = 0;
        // 比赛开始阶段进行图像检测（可能会出错，有问题的话就放waitForStart下面再试试。
        while (opModeInInit()) {
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
                } else{
                    MISSION = 2;
                }
            }
            robot.driveStrafe(0.5,1,-90);
            robot.driveStrafe(0.5,-1,-90);
            sleep(1000);
            telemetry.addData("MISSION", "%4d", MISSION);
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.update();
            if(runtime.seconds()>10)MISSION=2;
        }

 */
// endregion
