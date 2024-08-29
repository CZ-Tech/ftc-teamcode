/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision.pipeline;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class DetectionProcessorAdvanced implements VisionProcessor {


    Mat thresholdMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    Mat lft;
    Mat mdl;
    Mat rit;
    public int result = 0;

    private Mat ycrcbMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();

    public int teamColor = 2; // RED=1 BLUE = 2
    Rect rect1 = new Rect(5, 240, 150, 120);
    Rect rect2 = new Rect(250, 220, 150, 120);
    Rect rect3 = new Rect(480, 240, 150, 120);

    public void setTeamColor(int color) {
        teamColor = color;
    }

    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 255, 255);
    public int threshold = 150;
    Mat output = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public void setThreshold(int threshold) {
        this.threshold = threshold;
    }

    enum Stage {
        THRESHOLD,
        //        CONTOURS_OVERLAYED_ON_FRAME,
        Masked_Image,
        RAW_IMAGE;
    }


    public Stage stageToRenderToViewport = Stage.RAW_IMAGE;
    public Stage[] stages = Stage.values();

    private Telemetry telemetry;

    public DetectionProcessorAdvanced(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public void onViewportTapped() {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {

        contoursList.clear();
        input.copyTo(output);

        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */

        Imgproc.cvtColor(output, output, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(output, ycrcbMat, teamColor);
        Imgproc.threshold(ycrcbMat, thresholdMat, threshold, 255, Imgproc.THRESH_BINARY);
//        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//        numContoursFound = contoursList.size();
//        input.copyTo(contoursOnFrameMat);
//        Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);
//        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        if (teamColor == 1) {
            lower = new Scalar(0, 150, 0);
        }
        if (teamColor == 2) {
            lower = new Scalar(0, 0, 150);
        }
        Core.inRange(output, lower, upper, binaryMat);
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);
//        maskedInputMat.copyTo(output);

        lft = thresholdMat.submat(rect1);
        mdl = thresholdMat.submat(rect2);
        rit = thresholdMat.submat(rect3);

        //Imgproc.cvtColor(input, input, colorSpace.cvtCode);
        //Core.inRange(input, lower, upper, binaryMat);

        int c1 = Core.countNonZero(lft);
        int c2 = Core.countNonZero(mdl);
        int c3 = Core.countNonZero(rit);

        telemetry.addData("TeamColor=", teamColor == 1 ? "🔴" : "🔵");
        telemetry.addData("Threshold=", threshold);
        telemetry.addData("W1=", c1);
        telemetry.addData("W2=", c2);
        telemetry.addData("W3=", c3);

        if (c1 >= c2 && c1 >= c3) {
            this.result = 1;
            telemetry.addData("Signal", "Place:C1");
        } else if (c2 >= c1 && c2 >= c3) {
            this.result = 2;
            telemetry.addData("Signal", "Place:C2");
        } else if (c3 >= c1 && c3 >= c2) {
            this.result = 3;
            telemetry.addData("Signal", "Place:C3");

        }
        telemetry.addData("[Stage]", stageToRenderToViewport);
//        telemetry.addData("[Found Contours]", "%d", numContoursFound);
//        telemetry.update();

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                thresholdMat.copyTo(input);
            }

//            case CONTOURS_OVERLAYED_ON_FRAME:
//            {
//                return contoursOnFrameMat;
//            }

            case RAW_IMAGE: {
                 input.copyTo(input);
            }

            case Masked_Image: {
                maskedInputMat.copyTo(input);
            }

            default: {
//                    return input;
            }

        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Cool feature: This method is used for drawing annotations onto
        // the displayed image, e.g outlining and indicating which objects
        // are being detected on the screen, using a GPU and high quality
        // graphics Canvas which allow for crisp quality shapes.
        // Create a Paint object to set drawing attributes
        Paint paint = new Paint();
        paint.setColor(teamColor == 1 ? Color.RED : Color.BLUE);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3);


// Draw a red rectangle on the Canvas


        canvas.drawRect(makeGraphicsRect(rect1, scaleBmpPxToCanvasPx), paint);
        canvas.drawRect(makeGraphicsRect(rect2, scaleBmpPxToCanvasPx), paint);
        canvas.drawRect(makeGraphicsRect(rect3, scaleBmpPxToCanvasPx), paint);
        paint.setColor(Color.GREEN);
        if (result == 1)
            canvas.drawRect(makeGraphicsRect(rect1, scaleBmpPxToCanvasPx), paint);
        if (result == 2)
            canvas.drawRect(makeGraphicsRect(rect2, scaleBmpPxToCanvasPx), paint);
        if (result == 3)
            canvas.drawRect(makeGraphicsRect(rect3, scaleBmpPxToCanvasPx), paint);


    }

    public int getResult() {
        return this.result;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

}