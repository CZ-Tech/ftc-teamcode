package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;


@Disabled
@Config
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends LinearOpMode {
    public Robot robot = new Robot();

    public static int id = 20;

    @Override
    public void runOpMode(){
        robot.init(this);
        robot.telemetry.addLine("Wait for start");
        robot.telemetry.update();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(opModeIsActive()){
//            int tag = robot.visionLimelight.getTargetAprilTag();
//            switch (tag){
//                case 21: robot.telemetry.addData("AprilTag", "🟢🟣🟣");break;
//                case 22: robot.telemetry.addData("AprilTag", "🟣🟢🟣");break;
//                case 23: robot.telemetry.addData("AprilTag", "🟣🟣🟢");break;
//                default: robot.telemetry.addData("AprilTag", "Not Found");break;
//            }
            double[] result = robot.visionLimelight.getAprilTagResults(id);

            robot.telemetry.addData("tx", result[6]);
            robot.telemetry.addData("ty", result[7]);
            robot.telemetry.addData("dis", result[10]);

            robot.telemetry.update();
        }
    }
}

/*
1.20, 17
1.30, 15.5
1.40, 14
1.50, 12.6
1.80, 10.2
2.1, 5.1
2.2, 4.7
2.3, 4.65
 */