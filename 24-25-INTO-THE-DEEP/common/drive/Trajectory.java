package org.firstinspires.ftc.teamcode.common.drive;

import static org.firstinspires.ftc.teamcode.common.Globals.ODO_COUNTS_PER_INCH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

//@Confg
public class Trajectory {
    //region Dashboard
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    //endregion

    //region Point Details
    public double[] startPoint;
    public double lt, lx, ly, ldx, ldy; //Last position
    public double t, x, y, dx, dy; //Present position
    public Runnable fn; //Present function

    //Constants
    public static double SPEED = 0.5;
    public static double P_DRIVE_TURN_GAIN = 0.2;
    public static double P_DRIVE_STRAIGHT_GAIN = 0.2;
    public static double TURN_GAIN = 0.2;

    public ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    private double gotoX, gotoY;

    public enum TrajectoryMode{
        DOT,
        VELOCITY
    }
    //endregion

    //region Route Generator
    private double[] spline_fit(double x0, double dx0, double x1, double dx1) {
        double a = 2 * x0 + dx0 - 2 * x1 + dx1;
        double b = -3 * x0 - 2 * dx0 + 3 * x1 - dx1;
        double c = dx0;
        double d = x0;
        return new double[]{a, b, c, d};
    }

    private double spline_get(double[] spline, double u) {
        return spline[0] * u * u * u + spline[1] * u * u + spline[2] * u + spline[3];
    }
    //endregion

    //region Initialization
    public Trajectory(Robot robot){
        this.robot = robot;
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Trajectory reset(){
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odoDrivetrain.encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDrivetrain.encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDrivetrain.encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return this;
    }
    //endregion

    //region Start Robot
    public Trajectory startMove(){
        this.reset();
        return this.startMove(0, 0, 0, 0, () -> {});
    }

    public Trajectory startMove(Runnable fn){
        return this.startMove(0, 0, 0, 0, fn);
    }

    public Trajectory startMove(double n1, double n2, TrajectoryMode mode){
        return this.startMove(n1, n2, mode, () -> {});
    }

    public Trajectory startMove(double n1, double n2, TrajectoryMode mode, Runnable fn){
        if (mode == TrajectoryMode.DOT)
            return this.startMove(n1, n2, 0, 0, fn);
        return this.startMove(0, 0, n1, n2, fn);
    }

    public Trajectory startMove(double x, double y, double dx, double dy){
        return this.startMove(x, y, dx, dy, () -> {});
    }

    public Trajectory startMove(double[] point){
        return this.startMove(point[0], point[1], point[2], point[3]);
    }

    public Trajectory startMove(double x, double y, double dx, double dy, Runnable fn){
        //set start point
        startPoint = new double[]{x, y, dx, dy};
        this.t = 0;
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;
        this.robot.syncRun(fn);

        //set encoder mode
        this.reset();

        //start runtime
        runtime.reset();

        //FTCDashboard settings
        packet.fieldOverlay()
                .drawImage("/dash/into-the-deep.png", 0, 0, 144, 144, Math.toRadians(-90), 72, 72, false);

        return this;
    }
    //endregion

    //region Control
    public Trajectory stopMotor(){
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.odoDrivetrain.stopMotor();
        return this;
    }

    public Trajectory addPoint(double t, double x, double y, double dx, double dy){
        return this.addPoint(t, x, y, dx, dy, () -> {});
    }

    public Trajectory addPoint(double t, double x, double y, double dx, double dy, Runnable fn){
        return this.addPoint(t, x, y, dx, dy, 0, fn);
    }

    public Trajectory addPoint(double t, double x, double y, double dx, double dy, double heading){
        return this.addPoint(t, x, y, dx, dy, heading, () -> {});
    }

    public Trajectory addPoint(double t, double x, double y, double dx, double dy, double heading, Runnable fn){
        //record last point
        this.lt = this.t;
        this.lx = this.x;
        this.ly = this.y;
        this.ldx = this.dx;
        this.ldy = this.dy;
        //heading = Math.toRadians(heading);

        //present point
        this.t = t;
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;

        //main movement methods
        double x_error, y_error, turnSpeed;
        double now = runtime.seconds();
        while (now < this.t && now > this.lt && robot.opMode.opModeIsActive()){
            now = runtime.seconds();
            gotoX = spline_get(spline_fit(this.lx - startPoint[0], this.ldx, this.x - startPoint[0], this.dx), (now - this.lt) / (this.t - this.lt));
            gotoY = spline_get(spline_fit(this.ly - startPoint[1], this.ldy, this.y - startPoint[1], this.dy), (now - this.lt) / (this.t - this.lt));

            x_error = gotoX - (double) robot.odoDrivetrain.getLeftPosition() / ODO_COUNTS_PER_INCH; //Center
            y_error = gotoY - (double) robot.odoDrivetrain.getCenterPosition() / ODO_COUNTS_PER_INCH; //Left
            turnSpeed = Range.clip((robot.odoDrivetrain.getRightPosition() - robot.odoDrivetrain.getLeftPosition())
                    / ODO_COUNTS_PER_INCH * SPEED * P_DRIVE_TURN_GAIN,
                    -1, 1);
            //right - left
            if (!Globals.DEBUG) robot.odoDrivetrain.driveRobot(-x_error * P_DRIVE_STRAIGHT_GAIN * SPEED, y_error * P_DRIVE_STRAIGHT_GAIN * SPEED, -turnSpeed);

            packet.fieldOverlay().setRotation(-Math.toRadians(90)).setTranslation(4 + startPoint[1], -6 * 12 + 7 + startPoint[0]).setStroke("blue").setStrokeWidth(1).strokeRect(gotoX - 8.5, gotoY - 9, 17, 18);
            dashboard.sendTelemetryPacket(packet);
            if (Globals.DEBUG) robot.sleep(200);

//            robot.odometry.update();

            robot.telemetry.addData("goto X_POS", gotoX);
            robot.telemetry.addData("goto Y_POS", gotoY);
            robot.telemetry.addData("turnSpeed", turnSpeed);
            robot.telemetry.addLine();

            robot.telemetry.addData("X_POS", robot.odometry.X_Pos);
            robot.telemetry.addData("Y_POS", robot.odometry.Y_Pos);
            robot.telemetry.addData("Heading", robot.odometry.heading);
            robot.telemetry.addLine();

            robot.telemetry.addData("Left", robot.odoDrivetrain.getLeftPosition()/(2000 / (1.88976378 * Math.PI)));
            robot.telemetry.addData("Right", robot.odoDrivetrain.getRightPosition()/(2000 / (1.88976378 * Math.PI)));
            robot.telemetry.addData("Middle", robot.odoDrivetrain.getCenterPosition()/(2000 / (1.88976378 * Math.PI)));
            robot.telemetry.addLine();

            robot.telemetry.addData("RunTime", now);
            robot.telemetry.addData("Running to time", this.t);

            robot.telemetry.update();
        }

        //run function
        if (!Globals.DEBUG || Globals.DEBUGRUNFUNC) this.robot.syncRun(fn);

        return this;
    }

    public Trajectory addVelocity(double dx, double dy){
        return this.addPoint(this.t, this.x, this.y, dx, dy);
    }

    public Trajectory addFunc(Runnable fn){
        return this.addPoint(this.t, this.x, this.y, this.dx, this.dy, fn);
    }

    public Trajectory addTime(double toTime){
        return this.addTime(toTime, () -> {});
    }

    public Trajectory addTime(double toTime, Runnable fn){
        return this.addPoint(toTime, this.x, this.y, this.dx, this.dy, fn);
    }
    //endregion

    public double[] getStartPoint(){
        return this.startPoint;
    }

    public double[] getGotoPoint(){
        return new double[]{this.x, this.y, this.dx, this.dy};
    }

    public double[] getPreviousPoint(){
        return new double[]{this.lx, this.ly, this.ldx, this.ldy};
    }
}