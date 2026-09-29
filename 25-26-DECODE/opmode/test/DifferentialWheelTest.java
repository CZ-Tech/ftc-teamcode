package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Robot;

@Config
@Disabled
@TeleOp(name = "DifferentialTest", group = "Test")
public class DifferentialWheelTest extends LinearOpMode {
    Robot robot = new Robot();

    public static double startRPM = 3000, left = -500, right = 500;

    @Override
    public void runOpMode(){
        robot.init(this);

        waitForStart();

//        double leftOffset = 0, rightOffset = 0;
        Gamepad preGamepad1 = new Gamepad();
        preGamepad1.copy(gamepad1);
        while(opModeIsActive()){

            robot.gamepad1.update();
//            if (!preGamepad1.y && gamepad1.y) rightOffset += 100;
//            if (!preGamepad1.a && gamepad1.a) rightOffset -= 100;
//
//            if (!preGamepad1.dpad_up && gamepad1.dpad_up) leftOffset += 100;
//            if (!preGamepad1.dpad_down && gamepad1.dpad_down) leftOffset -= 100;

            if (gamepad1.ps) robot.subsystem.shooter.shoot(0, startRPM + left, startRPM + right);
            else robot.subsystem.shooter.stop();

            robot.telemetry.addData("leftOffset", left);
            robot.telemetry.addData("rightOffset", right);
            robot.telemetry.addData("leftRPM", startRPM + left);
            robot.telemetry.addData("rightRPM", startRPM + right);
            robot.telemetry.addData("左发射速度",robot.subsystem.shooter.left.getVelocity()/28*60);
            robot.telemetry.addData("右发射速度",robot.subsystem.shooter.right.getVelocity()/28*60);
            robot.telemetry.update();
            preGamepad1.copy(gamepad1);
        }
    }
}
