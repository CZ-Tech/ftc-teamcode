package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;

@TeleOp(name = "MotorTest2", group = "Test")
public class MotorTest2 extends LinearOpMode {

    Robot robot = new Robot();
    @Override
    public void runOpMode(){

        robot.teamColor = Alliance.BLUE;

        robot.init(this);



        waitForStart();

        robot.subsystem.shooter.setPower(1.0);

        while(opModeIsActive()) {


            robot.odoDrivetrain.driveRobotFieldCentricWithTeleOpHeadReset(
                    gamepad1.right_trigger > 0.5 ? 0.4 * -gamepad1.left_stick_y : -gamepad1.left_stick_y,
                    gamepad1.right_trigger > 0.5 ? 0.4 * gamepad1.left_stick_x : gamepad1.left_stick_x,
                    gamepad1.right_trigger > 0.5 ? 0.4 * gamepad1.right_stick_x : gamepad1.right_stick_x
            );

            robot.telemetry.addData("左发射速度",robot.subsystem.shooter.left.getVelocity()/28*60);
            robot.telemetry.addData("右发射速度",robot.subsystem.shooter.right.getVelocity()/28*60);

            robot.telemetry.update();
        }

        robot.subsystem.shooter.stop();

//        robot.limelight.stop();
    }
}

/*
1.403 3100
1.24 3000
1.50 3180
2.07 3500
2.075 3570
 */