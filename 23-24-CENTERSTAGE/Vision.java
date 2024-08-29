package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.concurrent.TimeUnit;

public class Vision {
    private final RobotHardware robot;
    public VisionPortal portal;
    private final Telemetry telemetry;
    private final LinearOpMode myOpMode;

    public Vision(RobotHardware robot) {
        this.robot = robot;
        myOpMode = robot.myOpMode;
        telemetry = myOpMode.telemetry;
    }

    public void closeVision() {
        portal.close();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (portal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!myOpMode.isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                robot.myOpMode.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested()) {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpMode.sleep(20);
        }
    }

    public void setCameraMode(ExposureControl.Mode mode) {
        while (!myOpMode.isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            robot.myOpMode.sleep(20);
        }
        portal
                .getCameraControl(ExposureControl.class)
                .setMode(mode);
    }

    public void init(VisionProcessor... processor) {
        portal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(1280, 720))
                .addProcessors(processor)
                .build();
    }
}
