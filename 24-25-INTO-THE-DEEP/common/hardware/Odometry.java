package org.firstinspires.ftc.teamcode.common.hardware;

import static org.firstinspires.ftc.teamcode.common.Globals.ODO_COUNTS_PER_INCH;

import static java.lang.Math.sin;


import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Matrix;
import org.firstinspires.ftc.teamcode.common.util.MatrixEx;

//@Confg
public class Odometry {

    public static double LATERAL_DISTANCE = 11.4;  //TODO:调试此值，让机器走准
    public static double FORWARD_OFFSET = 6.14;  //TODO:调试此值，让机器走准
    private final Robot robot;
    private final Matrix matrix = new Matrix();
    private final MatrixEx k1 = new MatrixEx(new double[][]{{0, 0}, {0, 0}});
    private final MatrixEx k2 = new MatrixEx(new double[][]{{0, 0}, {0, 0}});
    private final MatrixEx temp = new MatrixEx(new double[][]{{0}, {0}});
    private final MatrixEx result = new MatrixEx(new double[][]{{0}, {0}});

//  +++++++++++++++++++++++++++++++
//  +  |||                   |||  +
//  +  ||| LATERAL_DISTANCE  |||  +
//  +     <----------------->     +
//  +             ● Center        +        FIXME:Center指的是机器的旋转中心
//  +             |               +
//  +             | FORWARD_OFFSET+
//  +             V               +
//  +           =====             +
//  +           =====             +
//  +++++++++++++++++++++++++++++++
    public double delta_x, delta_y, X_Pos, Y_Pos, heading;
    public DcMotorEx encoderLeft = null;
    public DcMotorEx encoderRight = null;
    public DcMotorEx encoderCenter = null;
    private double prevLeftEncoder, prevRightEncoder, prevCenterEncoder, phi, delta_middle, delta_perp_pos;
    private double[][] matrix1;
    private double[][] matrix2;
//    private double[] result;

    //  Control Hub 0 -> Center
//  Control Hub 1 -> Left
//  Control Hub 2 -> Right


    public Odometry(Robot robot) {
        this.robot = robot;
        encoderCenter = robot.hardwareMap.get(DcMotorEx.class, "lfmotor"); //original: armmotor
        encoderLeft = robot.hardwareMap.get(DcMotorEx.class, "rfmotor");   //original: lencoder
        encoderRight = robot.hardwareMap.get(DcMotorEx.class, "rrmotor");  //original: rencoder

//        encoderCenter.setDirection(DcMotorSimple.Direction.REVERSE);//F
//        encoderLeft.setDirection(DcMotorSimple.Direction.REVERSE);//R
//        encoderRight.setDirection(DcMotorSimple.Direction.FORWARD);//F

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevCenterEncoder = 0;
    }

    public void resetPos() {
        X_Pos = 0;
        Y_Pos = 0;
        delta_x = 0;
        delta_y = 0;
        heading = 0;
    }

    //使用时，把update()单独开一个线程搞。

    public void update2(){
        phi = Math.asin(getDeltaLeftPosition() - getDeltaRightPosition()) / LATERAL_DISTANCE / ODO_COUNTS_PER_INCH;
        delta_x = (getDeltaLeftPosition() + getDeltaRightPosition()) / ODO_COUNTS_PER_INCH * Math.cos(phi) / 2.0;
        delta_y = (FORWARD_OFFSET + Math.sin(phi) + getDeltaCenterPosition()) / ODO_COUNTS_PER_INCH * Math.cos(phi);

        heading += phi;
        X_Pos += delta_x;
        Y_Pos += delta_y;

        prevCenterEncoder = getCenterPosition();
        prevLeftEncoder = getLeftPosition();
        prevRightEncoder = getRightPosition();
    }

    public void update() {
        phi = (getDeltaLeftPosition() - getDeltaRightPosition()) / LATERAL_DISTANCE / ODO_COUNTS_PER_INCH; //FIXME:单位是弧度
        delta_middle = (getDeltaLeftPosition() + getDeltaRightPosition()) / 2 / ODO_COUNTS_PER_INCH;
        delta_perp_pos = getDeltaCenterPosition() / ODO_COUNTS_PER_INCH - FORWARD_OFFSET * phi; //imu没法返回一个double，暂时用惰轮直接计算。

        //TODO: 待测试。
        k2.set(new double[][]{
                {sin(phi) / phi, (Math.cos(phi) - 1) / phi},
                {(1 - Math.cos(phi)) / phi, sin(phi) / phi}
        });
        k1.set(new double[][]{
                {Math.cos(heading), -sin(heading)},
                {sin(heading), Math.cos(heading)}
        });
        temp.set(new double[][]{
                {delta_middle},
                {delta_perp_pos}
        });
        if (phi != 0) {
//            result = matrix.times(matrix2, matrix.times(matrix1, new double[]{delta_middle, delta_perp_pos}));
            result.set(k1.multiply(k2.multiply(temp)));
            delta_x = result.data[0][0];
            delta_y = result.data[1][0];
        } else {
            result.set(k1.multiply(temp));
            delta_x = result.data[0][0];
            delta_y = result.data[1][0];
        }

//        result.set(k1.multiply(k2.multiply(temp)));
//        delta_x = result.data[0][0];
//        delta_y = result.data[1][0];

//        delta_x = delta_middle * Math.cos(heading) - delta_perp_pos * Math.sin(heading);
//        delta_y = delta_middle * Math.sin(heading) + delta_perp_pos * Math.cos(heading);



        heading += phi;
        X_Pos += delta_x;
        Y_Pos += delta_y;

        prevCenterEncoder = getCenterPosition();
        prevLeftEncoder = getLeftPosition();
        prevRightEncoder = getRightPosition();
    }

    public double getDeltaLeftPosition() {
        return getLeftPosition() - this.prevLeftEncoder;
    }

    public double getDeltaRightPosition() {
        return getRightPosition() - this.prevRightEncoder;
    }

    public double getDeltaCenterPosition() {
        return getCenterPosition() - this.prevCenterEncoder;
    }

    public AngularVelocity getDeltaTheta() {
        return robot.imu.getRobotAngularVelocity(AngleUnit.RADIANS);
    }

    public double getCenterPosition(){
        return -encoderCenter.getCurrentPosition();
    }

    public double getLeftPosition(){
        return encoderLeft.getCurrentPosition();
    }

    public double getRightPosition(){
        return -encoderRight.getCurrentPosition();
    }

}
