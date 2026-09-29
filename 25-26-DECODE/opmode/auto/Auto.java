package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;


@Disabled
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        //Initialization
        robot.init(this);
        PinpointTrajectory trajectory = new PinpointTrajectory(robot);
        robot.opModeState = OpModeState.Auto;
        int rank = robot.visionLimelight.getTargetAprilTag();

        robot.telemetry.addData("Status", "Waiting for start");
        robot.telemetry.addData("ColourRank",rank);

        robot.telemetry.update();

        trajectory.reset();
        robot.odoDrivetrain.resetYaw();
        robot.command.stopTransporting();        //Wait for the 'Start' button pressed

        waitForStart();

        switch (rank){
            case 21:
            case 22:
            case 23:
            default:break;
        }

//        //Main methods
        //路径
        trajectory
                .setMode(PinpointTrajectory.Mode.SEPARATED)
                .startMove()

        ;


        while (opModeIsActive());

    }

    //Red alliance
    @Disabled
    @Autonomous(name = "Auto🔴", group = "Auto", preselectTeleOp = "Duo🔴")
    public static class AutoRed extends Auto {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    //Blue alliance
    @Disabled
    @Autonomous(name = "Auto🔵", group = "Auto", preselectTeleOp = "Duo🔵")
    public static class AutoBlue extends Auto {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            super.runOpMode();
        }
    }
}
