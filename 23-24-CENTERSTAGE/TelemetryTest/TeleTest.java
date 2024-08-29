package org.firstinspires.ftc.teamcode.TelemetryTest;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name = "AutoOpTemplate", group = "Test")
@Autonomous(name = "SyncTest", group = "Test")
public class TeleTest extends LinearOpMode {
    //RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    //public final LinearOpMode myOpMode;
    RobotHardware_ZYC robot = new RobotHardware_ZYC(this);

    @Override
    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init();
        waitForStart();
        robot.syncrun(
                () -> {
                    robot.sleep(1000);
                    telemetry.addData("test2", "1919810");
                    telemetry.update();
                },
                () -> {
                    robot.sleep(2000);
                    telemetry.addData("test3", "1145141919810");
                    telemetry.update();
                }
        )
        ;
        telemetry.addData("test1", "114514");
        telemetry.update();
        robot.sleep(3000);


//        Thread thread = new Thread(
//            () -> {
//                robot.sleep(1000);
//                telemetry.addData("test2", "1919810");
//                telemetry.update();
//            }
//        );
//        thread.start();
    }
}
