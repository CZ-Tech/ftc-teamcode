package org.firstinspires.ftc.teamcode.common.vision.pipeline;

import android.annotation.SuppressLint;
import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

import java.util.List;

@Disabled
@TeleOp(name = "PipelineVisionColorLocator", group = "Concept")
public class PipelineVisionColorLocator extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // 创建自定义视觉处理器
        ColorBlobVisionProcessor visionProcessor = new ColorBlobVisionProcessor();

        // 配置VisionPortal
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(visionProcessor)
                .setCameraResolution(new Size(320, 240))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            // 从处理器获取处理结果
            List<ColorBlobLocatorProcessor.Blob> filteredBlobs = visionProcessor.getFilteredBlobs();

            // 更新遥测数据
            telemetry.addLine(" Area Density Aspect  Center");
            for (ColorBlobLocatorProcessor.Blob b : filteredBlobs) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(),
                        (int) boxFit.center.x, (int) boxFit.center.y));
            }
            telemetry.update();
        }
    }

    /* 自定义视觉处理器实现 */
    public static class ColorBlobVisionProcessor implements VisionProcessor {
        private final ColorBlobLocatorProcessor colorBlobProcessor;
        private List<ColorBlobLocatorProcessor.Blob> filteredBlobs;

        public ColorBlobVisionProcessor() {
            // 初始化颜色斑点检测器
            colorBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.BLUE)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                    .setDrawContours(true)
                    .setBlurSize(5)
                    .build();
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            colorBlobProcessor.init(width, height, calibration);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // 处理图像帧
            colorBlobProcessor.processFrame(frame, captureTimeNanos);

            // 获取并过滤斑点
            List<ColorBlobLocatorProcessor.Blob> blobs = colorBlobProcessor.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
            filteredBlobs = blobs;

            return null; //
        }
        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // 传递绘制调用给底层处理器
            colorBlobProcessor.onDrawFrame(
                    canvas, onscreenWidth, onscreenHeight,
                    scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext
            );
        }


        public List<ColorBlobLocatorProcessor.Blob> getFilteredBlobs() {
            return filteredBlobs;
        }



    }
}