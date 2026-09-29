package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class VisionC270 {
    public Robot robot;
    public ColorBlobLocatorProcessor colorLocatorPurple, colorLocatorGreen;

    public VisionC270(Robot robot){
        this.robot = robot;

        colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(org.firstinspires.ftc.vision.opencv.ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.53, 0.5, 0.72, -0.2))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.53, 0.5, 0.72, -0.2))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        robot.vision.init(colorLocatorGreen, colorLocatorPurple);
    }

    /**
     * 1=purple; -1=green; 0=none
     * @return color
     */
    public int getColor(){
        List<ColorBlobLocatorProcessor.Blob> blobsGreen = colorLocatorGreen.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> blobsPurple = colorLocatorPurple.getBlobs();
        if (!blobsPurple.isEmpty()) return 1;
        else if (!blobsGreen.isEmpty()) return -1;
        return 0;
    }

    public List<ColorBlobLocatorProcessor.Blob> getGreenBlobs(){
        return colorLocatorGreen.getBlobs();
    }

    public List<ColorBlobLocatorProcessor.Blob> getPurpleBlobs(){
        return colorLocatorPurple.getBlobs();
    }
}
