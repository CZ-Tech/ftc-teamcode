//package org.firstinspires.ftc.teamcode.test;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgcodecs.Imgcodecs;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.videoio.VideoCapture;
//import org.opencv.highgui;
//
//public class TemplateMatching {
//    static { System.loadLibr
//        VideoCapture cap = new VideoCapture(0);
//        // 加载模板图像
//        Mat template = Imgcodecs.imread("/boot/OIP-C.jpg");
//
//        while (cap.isOpened()) {
//            Mat frame = new Mat();
//            if (!cap.read(frame)) {
//                break;ary(Core.NATIVE_LIBRARY_NAME); }
//
//            public static void main(String[] args) {
//                // 加载摄像头
//            }
//
//            // 进行模板匹配
//            Mat result = new Mat();
//            Imgproc.matchTemplate(frame, template, result, Imgproc.TM_CCOEFF_NORMED);
//
//            // 找到最佳匹配位置
//            Core.MinMaxLocResult mmr = Core.minMaxLoc(result);
//            Point matchLoc = mmr.maxLoc;
//            double matchValue = mmr.maxVal;
//
//            System.out.println("匹配相似度: " + matchValue);
//
//            // 在帧上绘制矩形
//            if (matchValue > 0.2) {
//                Rect rect = new Rect(matchLoc, template.size());
//                Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(0, 0, 255));
//            }
//
//            // 显示结果
//            HighGui.imshow("result", frame);
//            HighGui.waitKey(30);
//
//            if (matchValue > 0.2) {
//                // 这里应该是GPIO控制的代码，但需要使用Java GPIO库来实现
//                // ...
//                break;
//            }
//        }
//
//        // 释放资源和关闭窗口
//        cap.release();
//        HighGui.destroyAllWindows();
//    }
//}
