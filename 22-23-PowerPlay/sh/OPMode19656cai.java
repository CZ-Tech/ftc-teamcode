package org.firstinspires.ftc.teamcode.sh;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.HardwareManager;

import java.util.HashMap;

@TeleOp(name = "cai", group = "SH")
public class OPMode19656cai extends LinearOpMode {
    //region定义
    private DcMotor LFmotor;
    private DcMotor LRmotor;
    private DcMotor RFmotor;
    private DcMotor RRmotor;
    double power_LFmotor;
    double power_LRmotor;
    double power_RFmotor;
    double power_RRmotor;
    double power_mul = 0.6F;
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
    private ColorSensor color1, color2, color3;
    private int c1r, c1g, c1b, c1a, c2r, c2g, c2b, c2a, c3r, c3g, c3b, c3a;
    private String c1label, c2label, c3label;
    private DistanceSensor distance;
    private Status status = Status.Ground;
    private Status2 status2 = Status2.one;

    //endregion
//region 定义手柄上的摇杆上的拨动时的输出的数值
    double p1lx;
    double p1rx;
    double p1ly;

    //endregion

    @Override
    public void runOpMode() {
        // region 定义舵机
        HardwareManager manager = new HardwareManager(hardwareMap);
        Servo0 = hardwareMap.get(ServoImplEx.class, "s0");//大臂爪子翻折
        Servo1 = hardwareMap.get(ServoImplEx.class, "s1");//大翻折左侧爪子
        Servo2 = hardwareMap.get(ServoImplEx.class, "s2");//大翻折右侧爪子
        Servo3 = hardwareMap.get(ServoImplEx.class, "s3");//大臂翻折
        Servo4 = hardwareMap.get(ServoImplEx.class, "s4");//撑脚
        Servo5 = hardwareMap.get(ServoImplEx.class, "s5");//最外侧滑轨舵机
        Servo6 = hardwareMap.get(ServoImplEx.class, "s6");//中间滑轨舵机
        Servo7 = hardwareMap.get(ServoImplEx.class, "s7");//最内侧滑轨舵机
        Servo8 = hardwareMap.get(ServoImplEx.class, "s8");//滑轨爪子翻折
        Servo9 = hardwareMap.get(ServoImplEx.class, "s9");//滑轨爪子
        Servo10 = hardwareMap.get(ServoImplEx.class, "s10");//撑脚
        Servo11 = hardwareMap.get(ServoImplEx.class, "s11");//撑脚
        Servo0.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo3.setPwmRange(new PwmControl.PwmRange(1000, 2500));
        Servo4.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo5.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo6.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo7.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo8.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo9.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo10.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo11.setPwmRange(new PwmControl.PwmRange(500, 2500));
//endregion
        //region 定义颜色传感器
        color1 = hardwareMap.get(ColorSensor.class, "c1");//最外侧滑轨舵机颜色传感器
        color2 = hardwareMap.get(ColorSensor.class, "c2");//中间滑轨舵机颜色传感器
        color3 = hardwareMap.get(ColorSensor.class, "c3");//最内侧滑轨舵机颜色传感器
        //endregion
        // region 定义距离传感器
        distance = hardwareMap.get(DistanceSensor.class, "d1");
//endregion
        //region 定义电机
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
        // endregion

        // 小翻折不做了！
        Servo8.setPosition(0.45);
        while (opModeIsActive()) {
            //region 当按下a时前进方向为开口方向 当按下b后前进方向为大翻折方向
            if (gamepad1.a) {
                status2 = Status2.one;
            }
            if (gamepad1.b) {
                status2 = Status2.two;
            }
            if (status2 == Status2.one) {
                p1lx = gamepad2.right_stick_x;
                p1ly = -gamepad2.right_stick_y;
                //转向补偿
                p1rx = 0.8 * MathUtils.clamp((gamepad2.left_stick_x * (0.4 * gamepad2.right_stick_x*gamepad2.right_stick_x +1))
                        * (0.4 * gamepad2.left_stick_y*gamepad2.left_stick_y +1), -1, 1);


                power_LFmotor = Range.clip((p1ly + p1lx + p1rx), -1, 1);
                power_RFmotor = Range.clip((p1ly - p1lx - p1rx), -1, 1);
                power_LRmotor = Range.clip((p1ly - p1lx + p1rx), -1, 1);
                power_RRmotor = Range.clip((p1ly + p1lx - p1rx), -1, 1);

                LFmotor.setPower(power_LFmotor * -0.6F);
                RFmotor.setPower(power_RFmotor * -0.6F);
                LRmotor.setPower(power_LRmotor * -0.6F);
                RRmotor.setPower(power_RRmotor * -0.6F);
            }

            if (status2 == status2.two) {
                p1lx = gamepad2.right_stick_x;
                p1ly = -gamepad2.right_stick_y;
                //转向补偿
                p1rx = 0.8 * MathUtils.clamp((gamepad2.left_stick_x * (0.4 * gamepad2.right_stick_x*gamepad2.right_stick_x +1))
                        * (0.4 * gamepad2.left_stick_y*gamepad2.left_stick_y +1), -1, 1);
                power_LFmotor = Range.clip((p1ly + p1lx + p1rx), -1, 1);
                power_RFmotor = Range.clip((p1ly - p1lx - p1rx), -1, 1);
                power_LRmotor = Range.clip((p1ly - p1lx + p1rx), -1, 1);
                power_RRmotor = Range.clip((p1ly + p1lx - p1rx), -1, 1);

                LFmotor.setPower(power_LFmotor * 0.6F);
                RFmotor.setPower(power_RFmotor * 0.6F);
                LRmotor.setPower(power_LRmotor * 0.6F);
                RRmotor.setPower(power_RRmotor * 0.6F);
            }
            //endregion

            //region 获取颜色传感器r,g,b
            c1r = color1.red();
            c1g = color1.green();
            c1b = color1.blue();
            c1a = color1.alpha();
            c2r = color2.red();
            c2g = color2.green();
            c2b = color2.blue();
            c2a = color2.alpha();
            c3r = color3.red();
            c3g = color3.green();
            c3b = color3.blue();
            c3a = color3.alpha();
            c1label = getClosestColor1(c1r, c1g, c1b);
            c2label = getClosestColor2(c2r, c2g, c2b);
            c3label = getClosestColor3(c3r, c3g, c3b);
            //endregion

//            //region 当按下左肩键时滑轨上升，当按下右肩键时滑轨下降
            if (gamepad2.left_bumper) {
                status = status.Operation;
                if ("high".equals(c1label)) {
                    Servo5.setPosition(0.5);
                } else {
                    Servo5.setPosition(1);
                }
                if ("high".equals(c2label)) {
                    Servo6.setPosition(0.49);
                } else {
                    Servo6.setPosition(0);
                }
                if ("high".equals(c3label)) {
                    Servo7.setPosition(0.5);
                } else {
                    Servo7.setPosition(1);
                }
            } else if (gamepad2.right_bumper) {
                status = status.Operation;
                if (c1a < 500){
                    Servo5.setPosition(0.5);
                } else {
                    Servo5.setPosition(0);
                }
                if (c2a <1000 ) {
                    Servo6.setPosition(0.49);
                } else {
                    Servo6.setPosition(1);
                }
                if ( c3a < 8 ) {
                    Servo7.setPosition(0.5);
                } else {
                    Servo7.setPosition(0);
                }
//
            } else if (status == status.Operation) {
                Servo5.setPosition(0.5);
                Servo6.setPosition(0.49);
                Servo7.setPosition(0.5);
            }
//            //endregion

            //region 当按下对应按键后 设定相应状态 舵机上升 直到颜色传感器检测到相应颜色后停止
            if (gamepad2.x) {
                Servo5.setPosition(1);
                Servo6.setPosition(0);
                Servo7.setPosition(1);
                status = Status.Low;
            }
            if (status == Status.Low && "low".equals(c1label)) {
                Servo5.setPosition(0.5);
            }
            if (status == Status.Low && "low".equals(c2label)) {
                Servo6.setPosition(0.49);
            }
            if (status == Status.Low && "low".equals(c3label)) {
                Servo7.setPosition(0.5);
            }
            if (gamepad2.y) {
                Servo5.setPosition(1);
                Servo6.setPosition(0);
                Servo7.setPosition(1);
                status = Status.Middle;
            }
            if (status == Status.Middle && "middle".equals(c1label)) {
                Servo5.setPosition(0.5);
            }
            if (status == Status.Middle && "middle".equals(c2label)) {
                Servo6.setPosition(0.49);
            }
            if (status == Status.Middle && "middle".equals(c3label)) {
                Servo7.setPosition(0.5);
            }
            if (gamepad2.b) {
                Servo5.setPosition(1);
                Servo6.setPosition(0);
                Servo7.setPosition(1);
                status = Status.High;
            }
            if (status == Status.High && "high".equals(c1label)) {
                Servo5.setPosition(0.5);
            }
            if (status == Status.High && "high".equals(c2label)) {
                Servo6.setPosition(0.49);
            }
            if (status == Status.High && "high".equals(c3label)) {
                Servo7.setPosition(0.5);
            }
            //endregion
//region 当按下按键时设定状态 舵机下降 直到颜色传感器检测到相应颜色后停止
            if (gamepad2.a) {
                Servo5.setPosition(0);
                Servo6.setPosition(1);
                Servo7.setPosition(0);
                status = Status.Ground;
            }
//            System.out.println("======================Auto19656_22======================");
//            System.out.printf("======================a %d,%d,%d %n", c1a,c2a,c3a);
//            System.out.printf("======================a %d,%d,%d %n", c1r,c2r,c3r);
//            System.out.printf("======================a %d,%d,%d %n", c1g,c2g,c3g);
//            System.out.printf("======================a %d,%d,%d %n", c1b,c2b,c3b);
//            System.out.printf("======================label %s,%s,%s %n", c1label,c2label,c3label);
            telemetry.addData("Color1", "%s %d %d %d %d",c1label, c1r,c1g,c1b,c1a);
            telemetry.addData("Color2", "%s %d %d %d %d",c2label, c2r,c2g,c2b,c2a);
            telemetry.addData("Color3", "%s %d %d %d %d",c3label, c3r,c3g,c3b,c3a);

            telemetry.update();

            if (c1a<500&&status== Status.Ground) {
                Servo5.setPosition(0.5);

            }
            if (c2a<1000&&status== Status.Ground) {
                Servo6.setPosition(0.49);
            }
            if (c3a<8&&status== Status.Ground) {
                Servo7.setPosition(0.5);
            }
//endregion

            //endregion
            if (gamepad2.right_trigger>0.5) {
                Servo9.setPosition(0.75);
            } else {
                Servo9.setPosition(1);
            }
        }
    }

    enum Status {Ground, Low, Middle, High, JiaoJie, Operation}

    ;

    enum Status2 {one, two}

    ;

    //region 将r，g，b变为三维坐标向量 测定与目标向量夹角 选取最接近的目标向量为结果
    private static final HashMap<String, int[]> COLOR1_VECTORS = new HashMap<String, int[]>() {{
//        put("ground", new int[]{179, 313, 274});
        put("low", new int[]{1350, 1220, 920});
        put("middle", new int[]{1950, 3150, 1350});
        put("high", new int[]{450, 900, 1500});
        put("white", new int[]{1740, 3000, 2650});
        //put("Jiaojie", new int[]{0, 0, 255});
    }};
    private static final HashMap<String, int[]> COLOR2_VECTORS = new HashMap<String, int[]>() {{
//        put("ground", new int[]{69, 77, 193});
        put("low", new int[]{3260, 1220, 1300});
        put("middle", new int[]{1928, 1615, 855});
        put("high", new int[]{166, 334, 528});
        put("white", new int[]{4990, 5600, 5140});
        //put("Jiaojie", new int[]{0, 0, 255});
    }};

    private static final HashMap<String, int[]> COLOR3_VECTORS = new HashMap<String, int[]>() {{
//        put("ground", new int[]{2, 5, 4});
        put("low", new int[]{500, 213, 96});
        put("middle", new int[]{990, 1620, 485});
        put("high", new int[]{57, 161, 590});
        put("white", new int[]{970, 1710, 1680});
        //put("Jiaojie", new int[]{0, 0, 255});
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
    //endregion
}