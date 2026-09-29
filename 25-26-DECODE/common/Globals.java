package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.vision.ColorRange;

@Config
public class Globals {
    public static boolean DEBUG = false;


    // 反注释下方代码进入调试模式，用于测试每个电机方向。
    // Globals.DEBUG = true;
    // 一号手柄按住share键或者back键测试
    /**
     * Xbox/PS4 Button - Motor
     * X / ▢         - Left  Front
     * Y / Δ         - Right Front
     * B / O         - Right Back
     * A / X         - Left  Back
     * The buttons are mapped to match the wheels spatially if you
     * were to rotate the gamepad 45deg°. x/square is the front left
     * ________        and each button corresponds to the wheel as you go clockwise
     * / ______ \
     * ------------.-'   _  '-..+              Front of Bot
     * /   _  ( Y )  _  \                  ^
     * |  ( X )  _  ( B ) |      Left Front  \    Right Front
     * ___  '.      ( A )     /|       Wheel       \       Wheel
     * .'    '.    '-._____.-'  .'       (x/▢)        \       (Y/Δ)
     * |       |                 |                      \
     * '.___.' '.               |          Left Back    \       Right Back
     * '.             /             Wheel       \       Wheel
     * \.          .'              (A/X)        \       (B/O)
     * \________/
     * https://rr.brott.dev/docs/v1-0/tuning/
     */

    //底盘电机名称
    public static String LeftFrontMotor = "lfm";  // ch0
    public static String RightFrontMotor = "rfm"; // ch1
    public static String LeftBackMotor = "lbm";  // ch2
    public static String RightBackMotor = "rbm";  // ch3

    //摄像头
//    public static String limelight = "Webcam 1";
    public static String logiC270 = "Webcam 1";
    //发射器
    public static String leftShooter = "ls";//eh 2
    public static String rightShooter = "rs";//eh 3
    //Intaker
    public static String leftArm = "ila";
    public static String rightArm = "ira";
    public static String intaker = "intaker";//eh 0
    //传送带
    public static String leftBelt = "lb";
    public static String rightBelt = "rb";
    public static String belt = "belt";//eh 1
    //门
    public static String doorController = "dc";
    public static String doorWheel = "dw";  // servo hub 5
    //分拣舵机
    public static String classifierServo = "cls";  // null

//    public static DcMotor.Direction LeftFrontMotorDirection = DcMotor.Direction.FORWARD;
//    public static DcMotor.Direction RightFrontMotoDirection = DcMotor.Direction.FORWARD;
//    public static DcMotor.Direction LeftBackMotorDirection = DcMotor.Direction.FORWARD;
//    public static DcMotor.Direction RightBackMotorDirection = DcMotor.Direction.FORWARD;

    public static DcMotor.Direction LeftFrontMotorDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction RightFrontMotoDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction LeftBackMotorDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction RightBackMotorDirection = DcMotor.Direction.REVERSE;

    //  设置里程计吊舱相对于跟踪点的位置偏移
    //  @param xOffset 检测前后运动的吊舱偏移量（毫米），中心左侧为正，右侧为负
    //  @param yOffset 检测左右运动的吊舱偏移量（毫米），中心前方为正，后方为负
    //  +++++++++++++++++++++++++++++++
    //  +                        |||  +
    //  +               xOffset  |||  +
    //  +             <--------->     +
    //  +             ● Center        +
    // FIXME:Center指的是机器的旋转中心
    //  +             |               +
    //  +             | yOffset       +
    //  +             V               +
    //  +           =====             +
    //  +           =====             +
    //  +++++++++++++++++++++++++++++++
    public static String odoName = "odo";  //ci2c0
    public static double odoXOffset = 0;
    public static double odoYOffset = -0.99872249;  // -95.99872249
    // TODO:确认自己使用的里程计类型
    public static GoBildaPinpointDriver.GoBildaOdometryPods odoType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    // 确认odo编码器方向，向前读数增加。
    public static GoBildaPinpointDriver.EncoderDirection odoXDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    // 确认odo编码器方向，向左读数增加。
    public static GoBildaPinpointDriver.EncoderDirection odoYDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    //电机相关
    public static double TURN_GAIN = 0.0005;
    public static double STEER_GAIN = 0.02;
    public static double diss_far = 3045;
    //    public static int[] diss_far_2 = {3580, 3100, 3125};
    public static double diss_near = 2715;

    public static double ShooterP = 30;  // 0.0003 0.0007
    public static double ShooterI = 0.0;
    public static double ShooterD = 0.0 ; // 0.00005
    public static double ShooterF = 13.422;

    // 软PID循环速率
    public static int PID_THREAD_Hz = 120;

    public static double Shoot_time = 2.15;

    public static double SHOOT_VEL_TOLERANCE = 120;
    public static double SHOOT_ACCEL_TOLERANCE = 500;
    public static int FAR_PWM_1 = 3450;
    public static int FAR_PWM_2 = 3200;
    //视觉相关
    public static ColorRange targetColor;

    //intaker电机0 belt电机1


    public static String ImuName = "imu";

    public static Pose2D RED_GOAL_POS = new Pose2D(DistanceUnit.INCH, -72, 72, AngleUnit.DEGREES, 0);
    public static Pose2D BLUE_GOAL_POS = new Pose2D(DistanceUnit.INCH, -72, -72, AngleUnit.DEGREES, 0);

    public static double HEAD_SHOOT_ANGLE_DEG = 180;  // 定义的车头和发射之间的夹角

    public static double SHOOT_DIS_OFFSET_MM = -650;
    public static double SHOOTER_K_LINEAR = -0.8;
    public static double SHOOTER_B_LINEAR = SHOOTER_K_LINEAR * -1800;  // 此项中的数字为直线零点，左加右减，x轴为距离(mm)
    public static double THIRD_BALL_COMPENSATION_FACTOR = 1.06;

    public static boolean UseVisionLocate = true;
}