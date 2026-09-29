package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.command.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;

@Disabled
@Autonomous(name = "JsonAuto")
public class JsonAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. Initialize robot hardware (your Robot class should include odo and odoDrivetrain)
        Robot robot = new Robot();
        robot.init(this);

        // 2. Instantiate trajectory controller
        PinpointTrajectory trajectory = new PinpointTrajectory(robot);

        waitForStart();

        if (opModeIsActive()) {
            // 3. Read trajectory JSON from HttpJsonService (persisted storage)
            String autoStr = HttpJsonService.getSavedJson("");

            // 4. Execute trajectory with marker tasks
            new TrajectoryLoader(trajectory)
                    .addMarkerTask("start_task", () -> {
                        // Code here will start at the beginning of the path
                        // (due to PinpointTrajectory mechanism, runs in a separate thread loop)
                        telemetry.addLine("Task started at the beginning!");
                        robot.command.transportArtifact();
                    })
                    .addMarkerTask("mid_task", () -> {
                        // Code here will start when reaching the middle marker point
                        telemetry.addLine("Reached the middle marker!");
                        robot.command.stopTransporting();
                    })
                    .execute(autoStr);
        }
    }
}