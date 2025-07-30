package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
//@Config
@TeleOp(name = "TestOp", group = "Debug")
public class TestOp extends LinearOpMode {

    Robot robot = new Robot();
    public AprilTagProcessor aprilTag;

    public void runOpMode() {
        robot.init(this);
        robot.opModeState = OpModeState.Duo;
//        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.getDetections();

        robot.subsystem.stretcher.RightExpandBack();
        robot.limelight.start();
        robot.subsystem.stretcher.LeftExpandBack();
        robot.subsystem.claw.up();
        robot.odoDrivetrain.stopMotor();

        robot.telemetry.addLine("Waiting for start");
        robot.telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.telemetry.addLine("Running...");

            robot.gamepad1.update()
                    .keyDown("dpad_left", () -> robot.limelight.setTargetColor(Alliance.RED))
                    .keyDown("dpad_up", () -> robot.limelight.setTargetColor(Alliance.YELLOW))
                    .keyDown("dpad_right", () -> robot.limelight.setTargetColor(Alliance.BLUE))
                    .keyDown("x", () -> robot.command.right_IngrabberEatForOpMode())
                    .keyDown("b", () -> robot.command.right_IngrabberPut())
                    .keyDown("a", () -> robot.command.rightClawRunForSamples())
//                    .keyDown("y", () -> robot.subsystem.stretcher.rightexpand.setPwmEnable())
                    .keyDown("left_bumper", () -> robot.command.turnForSample())
                    .keyDown("ps", () -> {
                        switch (robot.subsystem.grabber.stat){
                            case GRAB: robot.subsystem.grabber.release();break;
                            case RELEASE: robot.subsystem.grabber.grab();break;
                            default: break;
                        }
                    })//按下a松开/闭合爪子
                    .keyDown("right_bumper", () -> robot.subsystem.stretcher.rightStop())
            ;

            robot.limelight.update();
            robot.telemetry.addData("Stat", robot.limelight.getStat() ? "Found" : "Not Found");
            robot.telemetry.addData("X", robot.limelight.getCx());
            robot.telemetry.addData("Y", robot.limelight.getCy());
            robot.telemetry.addData("Tx", robot.limelight.getTx());
            robot.telemetry.addData("Ty", robot.limelight.getTy());
            robot.telemetry.addData("Cx", robot.limelight.getCx() - (int)robot.limelight.getWidth() / 2);
            robot.telemetry.addData("Cy", robot.limelight.getCy() - (int)robot.limelight.getHeight() / 2);
            robot.telemetry.addData("Angle", robot.limelight.getAngle());


//            robot.telemetry.addData("Heading", robot.odo.getPosition().getHeading(AngleUnit.DEGREES));
//            robot.telemetry.addData("X Position", robot.pinpointTrajectory.getX());
//            robot.telemetry.addData("Y Position", robot.pinpointTrajectory.getY());
//            robot.telemetry.addData("LeftMotorMoveCounts",robot.subsystem.arm.leftArmMotor.getCurrentPosition());
//            robot.telemetry.addData("RightMotorMoveCounts",robot.subsystem.arm.rightArmMotor.getCurrentPosition());
//            robot.telemetry.addData("LiftmotorMoveCounts",robot.subsystem.thrower.Liftmotor.getCurrentPosition());
            robot.telemetry.update();
        }
    }
}

