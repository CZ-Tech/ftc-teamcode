package org.firstinspires.ftc.teamcode.test;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class TelemetryPipeline implements VisionProcessor {

    Telemetry telemetry;

    public TelemetryPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int result = 0;
    public int teamcolor = 0; // RED=0 BLUE = 2

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public void setteam(String team) {
        if (team == "Red") {
            teamcolor = 0;
        } else {
            teamcolor = 2;
        }
    }

    Rect rect1 = new Rect(60, 110, 150, 150);
    Rect rect2 = new Rect(250, 60, 150, 150);
    Rect rect3 = new Rect(490, 110, 150, 150);

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        List<Mat> mv = new ArrayList<Mat>();
        Core.split(input, mv);
        input = mv.get(teamcolor);//Red
        //input = mv.get(2);//Blue
        Imgproc.threshold(input, input, 165, 255, Imgproc.THRESH_BINARY);

        Mat lft = input.submat(rect1);
        Mat mdl = input.submat(rect2);
        Mat rit = input.submat(rect3);
        int c1 = Core.countNonZero(lft);
        int c2 = Core.countNonZero(mdl);
        int c3 = Core.countNonZero(rit);
//        Imgproc.rectangle(input,
//                new Point(75,110),
//                new Point(165,200),
//                new Scalar(255,255,255)
//        );
//        Imgproc.rectangle(input,
//                new Point(300,100),
//                new Point(380,180),
//                new Scalar(255,255,255)
//        );
//        Imgproc.rectangle(input,
//                new Point(510,110),
//                new Point(600,200),
//                new Scalar(255,255,255)
//        );

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
//        telemetry.update();
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
        paint.setColor(teamcolor == 0 ? Color.RED : Color.BLUE);
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
