package org.firstinspires.ftc.teamcode.common.vision.processor;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

import java.util.ArrayList;
import java.util.Arrays;


public class ColorLocatorProcessor implements VisionProcessor {
    Telemetry telemetry;
    private Mat roiMat;
    private Mat roiMat_userColorSpace;
    public static int ERODE=3;
    private Mat mask = new Mat();
    private ColorRange colorRange;

    private RotatedRect rect;

    public ColorLocatorProcessor(ColorRange colorRange, Telemetry telemetry) {
        this.colorRange = colorRange;
        this.telemetry = telemetry;
    }
    public ColorLocatorProcessor(Telemetry telemetry) {
        this.colorRange = ColorRange.YELLOW;
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        colorRange= Globals.targetColor;
        roiMat = frame.clone();
        roiMat_userColorSpace = roiMat.clone();
        // 定义摄像头图像中的四个角点
        Point[] srcPoints = new Point[]{
                new Point(63, 1),  // 左上角
                new Point(232, 1),  // 右上角
                new Point(360, 221),  // 右下角
                new Point(-40, 221)   // 左下角
        };
        // 定义地面上四个角点的实际位置
        Point[] dstPoints = new Point[]{
                new Point(0, 0),       // 左上角
                new Point(320, 0),    // 右上角
                new Point(320, 240), // 右下角
                new Point(0, 240)     // 左下角
        };
        // 转换为Mat类型
        Mat srcMat = Converters.vector_Point2f_to_Mat(Arrays.asList(srcPoints));
        Mat dstMat = Converters.vector_Point2f_to_Mat(Arrays.asList(dstPoints));
        // 计算单应性矩阵
        Mat H = Imgproc.getPerspectiveTransform(srcMat, dstMat);
        // 创建输出图像
        Mat topViewImage = new Mat();
        // 进行透视变换
        Imgproc.warpPerspective(roiMat, roiMat, H, new Size(320, 240));
        // 色彩空间转换
        Imgproc.cvtColor(roiMat, roiMat_userColorSpace, Imgproc.COLOR_RGB2YCrCb);
        // 高斯模糊
        Imgproc.GaussianBlur(roiMat_userColorSpace, roiMat_userColorSpace, new Size(5, 5), 0);
        // 提取
        Core.inRange(roiMat_userColorSpace, colorRange.min, colorRange.max, mask);
        // 腐蚀
        Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(ERODE, ERODE)));
        // 膨胀
        Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

        // 找到所有轮廓
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask,
                contours,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        Imgproc.drawContours(roiMat, contours, -1, new Scalar(255), 2);

        // 找到最大轮廓
        double maxArea = 0;
        MatOfPoint2f largestContour = null;
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double area = Imgproc.contourArea(contour2f);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour2f;
            }
        }
        if (largestContour != null) {
            // 计算最小外接矩形
            rect = Imgproc.minAreaRect(largestContour);
//            telemetry.addData("rect center",rect.center.toString());
//            telemetry.update();
//            // 获取矩形的四个顶点
//            Point[] vertices = new Point[4];
//            rect.points(vertices);
//
//            // 绘制旋转矩形（连接四个顶点）
//            for (int i = 0; i < 4; i++) {
//                Imgproc.line(
//                        roiMat,                  // 目标图像
//                        vertices[i],            // 起点
//                        vertices[(i + 1) % 4],    // 终点（循环连接）
//                        new Scalar(0, 255, 0),  // 颜色（绿色）
//                        2                       // 线宽
//                );
//            }
        }


//        if (Globals.CAMERA_MASK) {
//            mask.copyTo(frame);
//        } else {
            roiMat.copyTo(frame);
//        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public RotatedRect getRect() {
        return this.rect;
    }
}
