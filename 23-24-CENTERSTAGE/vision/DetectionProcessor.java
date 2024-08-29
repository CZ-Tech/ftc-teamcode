package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.teamcode.utlity.AllianceColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class DetectionProcessor implements VisionProcessor {

    Telemetry telemetry;
    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 255, 255);


    public DetectionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int result = 0;
    public int teamColor = 2; // RED=1 BLUE = 2
    public int threshold = 150;
    Rect rect1 = new Rect(5, 240, 150, 120);
    Rect rect2 = new Rect(250, 220, 150, 120);
    Rect rect3 = new Rect(480, 240, 150, 120);

    Mat output;
    Mat lft;
    Mat mdl;
    Mat rit;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
//        output = new Mat(height,width,1);

    }


    public void setTeamColor(int color) {
        teamColor = color;
    }

    public void setThreshold(int threshold) {
        this.threshold = threshold;
    }


    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
//        input.copyTo(output);
//        output=input;

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(input, input, teamColor);
        Imgproc.threshold(input, input, threshold, 255, Imgproc.THRESH_BINARY);
        lft = input.submat(rect1);
        mdl = input.submat(rect2);
        rit = input.submat(rect3);

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
        /*        if (c1 >= 3500 && c2 < 3500 && c3 < 3500) {
         *            this.result = 1;
         *          telemetry.addData("Signal", "Place:C1");
         *       } else if (c2 >= 3500 && c1 < 3500 && c3 < 3500) {
         *           this.result = 2;
         *          telemetry.addData("Signal", "Place:C2");
         *       } else if (c3 >= 3500 && c1 < 3500 && c2 < 3500) {
         *           this.result = 3;
         *           telemetry.addData("Signal", "Place:C3");
         *       } else {
         *           this.result = 0;
         *           telemetry.addData("Signal", "Not Found");
         *       }
         *       telemetry.update();
         *       return null; // Return the input mat
         */
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
//       telemetry.update();
//        output.release();
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
