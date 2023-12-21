package org.firstinspires.ftc.teamcode.sh;

import android.graphics.Color;
import android.net.Network;
import android.print.PrintJobInfo;

import androidx.core.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.WebServer;
import com.qualcomm.robotcore.wifi.NetworkConnection;

import org.firstinspires.ftc.teamcode.util.HardwareManager;
import org.java_websocket.server.WebSocketServer;

import java.nio.channels.NetworkChannel;
import java.util.HashMap;

@TeleOp(name = "Manual", group = "SH")

public class HuaGuiYuShe extends LinearOpMode {
    private DcMotor LFmotor;
    private DcMotor LRmotor;
    private DcMotor RFmotor;
    private DcMotor RRmotor;

    private ServoImplEx Servo0;
    private ServoImplEx Servo1;
    private ServoImplEx Servo2;
    private ServoImplEx Servo3;
    private ServoImplEx Servo4;
    private ServoImplEx Servo5;
    private ServoImplEx Servo6;
    private ServoImplEx Servo7;
    private ServoImplEx Servo8;
    private ServoImplEx Servo9;
    private ServoImplEx Servo10;
    private ServoImplEx Servo11;
    private TouchSensor touch1, touch2;
    private ColorSensor  color1, color2, color3;
    private int c1r,c1g,c1b,c1a,c2r,c2g,c2b,c2a,c3r,c3g,c3b,c3a ;
    private DistanceSensor distance;
    private Status status = Status.Ground;

    @Override
    public void runOpMode() {
        HardwareManager manager = new HardwareManager(hardwareMap);
        Servo0 = hardwareMap.get(ServoImplEx.class, "s0");//大翻折上
        Servo1 = hardwareMap.get(ServoImplEx.class, "s1");//大翻折左
        Servo2 = hardwareMap.get(ServoImplEx.class, "s2");//大翻折右
        Servo3 = hardwareMap.get(ServoImplEx.class, "s3");//大翻折下
        Servo4 = hardwareMap.get(ServoImplEx.class, "s4");//
        Servo5 = hardwareMap.get(ServoImplEx.class, "s5");
        Servo6 = hardwareMap.get(ServoImplEx.class, "s6");//大翻折上
        Servo7 = hardwareMap.get(ServoImplEx.class, "s7");//大翻折左
        Servo8 = hardwareMap.get(ServoImplEx.class, "s8");//大翻折右
        Servo9 = hardwareMap.get(ServoImplEx.class, "s9");//大翻折下
        Servo10 = hardwareMap.get(ServoImplEx.class, "s10");//
        Servo11 = hardwareMap.get(ServoImplEx.class, "s11");
        Servo0.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo3.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo4.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo5.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo6.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo7.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo8.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo9.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo10.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo11.setPwmRange(new PwmControl.PwmRange(500, 2500));

        color1 = hardwareMap.get(ColorSensor.class, "c1");
        color2 = hardwareMap.get(ColorSensor.class, "c2");
        color3 = hardwareMap.get(ColorSensor.class, "c3");

        distance = hardwareMap.get(DistanceSensor.class, "d1");


        LFmotor = manager.getMotor(HardwareManager.Motor.LFmotor);
        LRmotor = manager.getMotor(HardwareManager.Motor.LRmotor);
        RFmotor = manager.getMotor(HardwareManager.Motor.RFmotor);
        RRmotor = manager.getMotor(HardwareManager.Motor.RRmotor);

        LFmotor.setDirection(DcMotor.Direction.FORWARD);
        LRmotor.setDirection(DcMotor.Direction.FORWARD);
        RFmotor.setDirection(DcMotor.Direction.REVERSE);
        RRmotor.setDirection(DcMotor.Direction.REVERSE);

        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
//此处由下至上为s5 s6 s7
//当按下按键时舵机向上运行，直到检测到色卡时停止
        while (opModeIsActive()) {
            c1r= color1.red();
            c1g=color1.green();
            c1b=color1.blue();
            c1a=color1.alpha();
            c2r= color2.red();
            c2g=color2.green();
            c2b=color2.blue();
            c2a= color2.alpha();
            c3r= color3.red();
            c3g=color3.green();
            c3b=color3.blue();
            c3a=color3.alpha();
            if (gamepad2.x) {
                Servo5.setPosition(1);
                Servo6.setPosition(0);
//                Servo7.setPosition(1);
                status = Status.Low;
            }
            if (status == Status.Low&&"low".equals(getClosestColor1(c1r,c1g,c1b)) ) {
                Servo5.setPosition(0.5);
            }
            if (status == Status.Low&&"low".equals(getClosestColor2(c2r,c2g,c2b))) {
                Servo6.setPosition(0.49);
            }
            if (status == Status.Low &&"low".equals(getClosestColor3(c3r,c3g,c3b))) {
//                Servo7.setPosition(0.5);
            }
            if (gamepad2.y) {
                Servo5.setPosition(1);
                Servo6.setPosition(0);
//                Servo7.setPosition(1);
                status = Status.Middle;
            }
            if (status == Status.Middle&&"middle".equals(getClosestColor1(c1r,c1g,c1b)) ) {
                Servo5.setPosition(0.5);
            }
            if (status == Status.Middle&&"middle".equals(getClosestColor2(c2r,c2g,c2b))) {
                Servo6.setPosition(0.49);
            }
            if (status == Status.Middle &&"middle".equals(getClosestColor3(c3r,c3g,c3b))) {
//                Servo7.setPosition(0.5);
            }
            if (gamepad2.b) {
                Servo5.setPosition(1);
                Servo6.setPosition(0);
//                Servo7.setPosition(1);
                status = Status.High;
            }
            if (status == Status.High&&"high".equals(getClosestColor1(c1r,c1g,c1b)) ) {
                Servo5.setPosition(0.5);
            }
            if (status == Status.High&&"high".equals(getClosestColor2(c2r,c2g,c2b))) {
                Servo6.setPosition(0.49);
            }
            if (status == Status.High &&"high".equals(getClosestColor3(c3r,c3g,c3b))) {
//                Servo7.setPosition(0.5);
            }
// 当按下按键时舵机向下运行，直到检测到色卡时停止
            if (gamepad2.a) {
                Servo5.setPosition(0);
                Servo6.setPosition(1);
//                Servo7.setPosition(0);
                status = Status.Ground;
            }
            if (status == Status.Ground &&"ground".equals(getClosestColor1(c1r,c1g,c1b))&&c1a<300) {
                Servo5.setPosition(0.5);
            }
            if (status == Status.Ground &&"ground".equals(getClosestColor2(c2r,c2g,c2b))&&c2a<1000&&c2g<800&&c2a<800){
                Servo6.setPosition(0.49);
            }
            if (status == Status.Ground &&"ground".equals(getClosestColor3(c3r,c3g,c3b))&&c3a<100&&c3g<100&&c3b<100) {
//                Servo7.setPosition(0.5);
            }

//此处Servo2指大翻折头部舵机 3指大翻折尾部舵机 4指大翻折爪子左 5指大翻折爪子右 6指小翻折爪子 7指小翻折舵机
// 当按下a后大翻折变为垂直状态
//            if (gamepad1.a) {
//                Servo2.setPosition(0.55);
//                Servo3.setPosition(0.5);
//            }
////当按下 b后大翻折变为水平状态
//            if (gamepad1.b) {
//                Servo2.setPosition(0.05);
//                Servo3.setPosition(0.975);
//            }
//////当按下x后自动交接 大翻折最终为水平状态
//            if (gamepad1.x) {
//                Servo0.setPosition(0);
//                Servo1.setPosition(0);
//                status = Status.JiaoJie;
//            }
//            if (color1.blue() > 300 && color2.blue() > 300 && color3.blue() > 300 && status == Status.JiaoJie) {
//                Servo0.setPosition(0.5);
//                Servo1.setPosition(0.5);
//                Servo4.setPosition(1);
//
//                Servo5.setPosition(0);
//                Servo7.setPosition(0.78);
//                Servo6.setPosition(0.75);
//                sleep(100);
//                Servo3.setPosition(0.475);
//                Servo2.setPosition(1);
//                sleep(100);
//                Servo6.setPosition(0.9);
//                sleep(100);
//                Servo4.setPosition(0.75);
//                Servo5.setPosition(0.25);
//                Servo6.setPosition(0);
//                Servo2.setPosition(0.55);
//                Servo3.setPosition(0.5);
////
//            }

        }


    }
    enum Status {Ground,Low,Middle,High,JiaoJie}


    private static final HashMap<String, int[]> COLOR1_VECTORS = new HashMap<String, int[]>() {{
        put("ground", new int[]{134, 232, 206});
        put("low", new int[]{1350, 1220, 920});
        put("middle", new int[]{1950, 3150, 1350});
        put("high", new int[]{450, 900, 1500});
        put("white", new int[]{1740, 3000, 2650});
    }};

    private static final HashMap<String, int[]> COLOR2_VECTORS = new HashMap<String, int[]>() {{
        put("ground", new int[]{60, 70, 60});
        put("low", new int[]{1245, 466, 412});
        put("middle", new int[]{1928, 1615, 855});
        put("high", new int[]{166, 334, 528});
        put("white", new int[]{1485, 1480, 1335});
    }};

    private static final HashMap<String, int[]> COLOR3_VECTORS = new HashMap<String, int[]>() {{
        put("ground", new int[]{25, 50, 50});
        put("low", new int[]{500, 213, 96});
        put("middle", new int[]{990, 1620, 485});
        put("high", new int[]{57, 161, 590});
        put("white", new int[]{970, 1710, 1680});
    }};
    public static String getClosestColor1(int r, int g, int b) {
        double minAngle = Double.MAX_VALUE;
        String closestColor = "";

        int[] inputVector = new int[]{r, g, b};
        for (String color : COLOR1_VECTORS.keySet()) {
            int[] colorVector = COLOR1_VECTORS.get(color);
            double angle = angleBetweenVectors(inputVector, colorVector);
            if (angle < minAngle) {
                minAngle = angle;
                closestColor = color;
            }
        }

        return closestColor;
    }

    public static String getClosestColor2(int r, int g, int b) {
        double minAngle = Double.MAX_VALUE;
        String closestColor = "";

        int[] inputVector = new int[]{r, g, b};
        for (String color : COLOR2_VECTORS.keySet()) {
            int[] colorVector = COLOR2_VECTORS.get(color);
            double angle = angleBetweenVectors(inputVector, colorVector);
            if (angle < minAngle) {
                minAngle = angle;
                closestColor = color;
            }
        }

        return closestColor;
    }
    public static String getClosestColor3(int r, int g, int b) {
        double minAngle = Double.MAX_VALUE;
        String closestColor = "";

        int[] inputVector = new int[]{r, g, b};
        for (String color : COLOR3_VECTORS.keySet()) {
            int[] colorVector = COLOR3_VECTORS.get(color);
            double angle = angleBetweenVectors(inputVector, colorVector);
            if (angle < minAngle) {
                minAngle = angle;
                closestColor = color;
            }
        }

        return closestColor;
    }
    private static double angleBetweenVectors(int[] v1, int[] v2) {
        double dotProduct = 0;
        double mag1 = 0;
        double mag2 = 0;

        for (int i = 0; i < v1.length; i++) {
            dotProduct += v1[i] * v2[i];
            mag1 += Math.pow(v1[i], 2);
            mag2 += Math.pow(v2[i], 2);
        }

        mag1 = Math.sqrt(mag1);
        mag2 = Math.sqrt(mag2);

        double cosTheta = dotProduct / (mag1 * mag2);
        return Math.acos(cosTheta);
    }

}