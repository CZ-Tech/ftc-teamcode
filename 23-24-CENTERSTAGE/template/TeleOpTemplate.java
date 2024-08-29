package org.firstinspires.ftc.teamcode.template;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

//@TeleOp(name = "TeleOpTemplate", group = "Test")
public class TeleOpTemplate extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();
        robot.resetYaw();
        waitForStart();
    }
}
