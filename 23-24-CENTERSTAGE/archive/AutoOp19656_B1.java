package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.Alliance;
import org.firstinspires.ftc.teamcode.vision.DetectionProcessor;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;

import java.util.ArrayList;

@Deprecated
@Autonomous(name = "🔵自动操控模式_B1", group = "Robot", preselectTeleOp = "手动阶段")
@Disabled
public class AutoOp19656_B1 extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    DetectionProcessor processor = new DetectionProcessor(telemetry);
    ArrayList nothing;

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

        if (mission == 1) robot.route.b11();
        if (mission == 2) robot.route.b12();
        if (mission == 3) robot.route.b13(nothing);
    }
}

// region 以下是老代码
//sleep(10000);
//        robot.G(-83);

// mission 0=未检测 1-左 2-中 3-右
//        vision.processFrame();
/*        int MISSION = 0;

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
*/

//telemetry.addData("MISSION", "%4d", MISSION);

//if(runtime.seconds()>10)MISSION=2;

//sleep(3000);
//        }


// Wait for the game to start (driver presses PLAY)


//        MISSION = 2;
//        mission = vision.result;
//        robot.closeVision();
//mission=2;
//        robot.sleep(10000);

//    public void putGroundPixel() {
//        robot
//                //启动飞轮放像素
//                .setLeftIntakePower(-1.0)
//                //等待0.5s
//                .sleep(500)
//                //关闭飞轮
//                .setLeftIntakePower(0)
//        ;
//    }
// endregion
