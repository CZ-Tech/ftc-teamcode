package org.firstinspires.ftc.teamcode.template;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

//@Autonomous(name = "AutoOpTemplate", group = "Test")
public class AutoOpTemplate extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();
        robot.resetYaw();
        waitForStart();
    }
}
