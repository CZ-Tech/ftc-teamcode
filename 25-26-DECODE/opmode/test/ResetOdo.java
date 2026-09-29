package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "ResetOdo", group = "Test")
@Config
public class ResetOdo extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        MixedOdo.isPoseInitialized = false;
        robot.init(this);
        telemetry.addLine("Odo has been reset! Next init needs AprilTag!");
        while (!MixedOdo.isPoseInitialized && !isStarted()) {
            robot.odo.update();
            telemetry.update();
        };
        waitForStart();
        while (opModeIsActive()) {
            robot.odo.update();
            telemetry.update();
        }
    }

}

