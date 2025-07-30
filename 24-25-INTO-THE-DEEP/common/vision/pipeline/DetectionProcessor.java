//package org.firstinspires.ftc.teamcode.common.vision.pipeline;
//
//import android.graphics.Bitmap;
//import android.graphics.Canvas;
//import android.graphics.Color;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.function.Consumer;
//import org.firstinspires.ftc.robotcore.external.function.Continuation;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
//import org.opencv.core.Mat;
//import org.opencv.core.Scalar;
//import org.firstinspires.ftc.vision.VisionProcessor;
//
//public class PredominantColorProcessor implements VisionProcessor {
//
//    private org.firstinspires.ftc.vision.opencv.PredominantColorProcessor colorSensor;
//    Telemetry telemetry;
//
//
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//        colorSensor = new org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Builder()
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
//                .setSwatches(
//                        org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Swatch.RED,
//                        org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Swatch.BLUE,
//                        org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Swatch.YELLOW,
//                        org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Swatch.BLACK,
//                        org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Swatch.WHITE)
//                .build();
//    }
//
//    @Override
//    public Object processFrame(Mat frame, long captureTimeNanos) {
//        // 处理帧并返回分析结果
//        colorSensor.processFrame(frame, captureTimeNanos); // 确保处理当前帧
//        return colorSensor.getAnalysis(); // 返回分析结果
//    }
//
//    @Override
//    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");
//        org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Result result = colorSensor.getAnalysis(); // 获取分析结果
//
//        telemetry.addData("Best Match:", result.closestSwatch);
//        telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
//        telemetry.update();
//    }
//
//
//}
