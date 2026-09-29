package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.command.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.command.SplineTracker;
import org.firstinspires.ftc.teamcode.common.command.SplineTrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.command.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;

/**
 * A dynamically registered OpMode that executes a saved JSON trajectory from
 * {@link HttpJsonService}. One instance is created per saved path name; the
 * instance is reused across OpMode runs (LinearOpMode supports this).
 *
 * <p>Registered via {@code HttpJsonService}'s {@code InstanceOpModeRegistrar}
 * so that adding/removing paths via the HTTP API immediately updates the
 * Driver Station OpMode list.</p>
 */
public class JsonPathOpMode extends LinearOpMode {

    private final String pathName;

    private static boolean useSplineTracker = true;

    /**
     * @param pathName the key used to look up the saved JSON in HttpJsonService
     */
    public JsonPathOpMode(String pathName) {
        this.pathName = pathName;
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(this);

        MixedOdo.isPoseInitialized = true;

        PinpointTrajectory trajectory = new PinpointTrajectory(robot);
        SplineTracker tracker = new SplineTracker(robot);

        waitForStart();

        if (opModeIsActive()) {
            String json = HttpJsonService.getSavedJson(pathName);
            if (json != null && !json.isEmpty()) {
                // Execute trajectory without marker tasks.
                // Marker names in the JSON are ignored — only the path is followed.
                if (useSplineTracker) new SplineTrajectoryLoader(tracker).execute(json);
                else new TrajectoryLoader(trajectory).execute(json);
            } else {
                robot.telemetry.addData("JsonPathOpMode", "No JSON found for path: " + pathName);
                robot.telemetry.update();
            }

            robot.odoDrivetrain.driveRobotFieldCentric(0,0,0);

            // Give async tasks (shooting, transport, etc.) time to finish
            // before the OpMode thread exits and tasks are killed.
            TaskLoopFrame.joinAllTask(30000);
        }

        TaskLoopFrame.stopAndClearAll();
    }
}
