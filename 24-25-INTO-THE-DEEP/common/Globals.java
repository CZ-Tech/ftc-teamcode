package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.common.vision.processor.ColorRange;

//@Config
public class Globals {
    // Rev HD Hex Motor (No Gearbox) : 28 counts/revolution
    // eg: GoBILDA 312 RPM (19.2:1) Yellow Jacket 537.6=28*19.2
    // 40:1 Rev 28*40=1120
    // 20:1 Rev 28*20=560
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as  needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    public static final double COUNTS_PER_MOTOR_REV = 19.2 * 28;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // 这里没有外部齿轮
    public static final double WHEEL_DIAMETER_INCHES = 4.00;     // 此时轮子直径为10.16cm
    public static final RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
    );//用于确定机器的朝向

    // 定义移动常量. 定义public类型以在opmode中调用
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double ODO_COUNTS_PER_INCH = 2000 / (1.88976378 * Math.PI); //13.26291192f

    //定义里程计相对于中心的偏差
    //148.027 -44.020
    public static final double X_OFFSET = 0;
    public static final double Y_OFFSET = -95.99872249;


    public static final double P_DRIVE_GAIN = 0.03;     // 更大的数值使它更加灵敏, 但会让它更不稳定
    public static final double P_STRAFE_GAIN = 0.025;   // 平移速度控制 "Gain".
    public static final double P_TURN_GAIN = 0.02;     // 更大的数值使它更加灵敏, 但会让它更不稳定

    public static double HEADING_THRESHOLD = 1;    // 此参数为理想偏向角度与实际偏向角度之间的差值。若设置的太小，机器就会一直不断地进行纠偏而影响后续程序执行，设置得太大就会让机器的姿态产生问题。
    public static boolean DEBUG=false;
    public static boolean DEBUGRUNFUNC = false;
    public static boolean DONOTRUNFUNC = false;
    public static boolean CAMERA_MASK = false;
    public static int VAR1=1900;
    public static int VAR2=100;
    public static ColorRange targetColor;
}