package org.firstinspires.ftc.teamcode.TelemetryTest;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TelemetryTest.GetMousePosition;
//@TeleOp(name = "TeleOpTemplate", group = "Test")
public class MouseTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
//    GetMousePosition position = new GetMousePosition();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init();
        robot.resetYaw();
//        position.onCreate();
        double mouse_x = 0.0;
        double mouse_y = 0.0;

//        View view = findViewById(R.id.yourView);
        waitForStart();

        

        while (opModeIsActive()) {
//            mouse_x = position.myView.position[0];

        }
    }


}
