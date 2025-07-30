package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name = "visionDriveStrafer", group = "Concept")
public class visionDriveStrafer extends LinearOpMode {

    Robot robot;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.init(this);
        robot.odo.update();

        if (gamepad1.share && gamepad1.options) {
            robot.odoDrivetrain.resetYaw();
            telemetry.addData("resetIMU", 1);
        } else {
            telemetry.addData("resetIMU", 0);
        }


        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        waitForStart();

        while (opModeIsActive()) {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

            List<Point> positions = new ArrayList<>();

            telemetry.addLine("Area Density Aspect Center");

            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect boxFit = b.getBoxFit();
                positions.add(boxFit.center);
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(),
                        (int) boxFit.center.x, (int) boxFit.center.y));
            }

          if (!positions.isEmpty()) {
               Point blobCenter = positions.get(0);
                double centerX = blobCenter.x;
                alignWithTarget(centerX);
           }

            telemetry.update();
            sleep(50);
        }
    }

    public void alignWithTarget(double centerX) {
        double targetX;

        if (centerX < 160) {
            targetX = 75;
        } else {
            targetX = 245;
        }

        double error = targetX - centerX;
        double speed = error * 0.01;

        speed = Range.clip(speed, -1.0, 1.0);

        if (Math.abs(error) < 5) {
            robot.pause();
        }


    }
}


