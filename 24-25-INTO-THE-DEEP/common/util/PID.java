package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

public class PID {
    private final double Kp;
    private final double Ki;
    private final double Kd;
    private final double Kf;
    private final double integralSumLimit;// = Double.POSITIVE_INFINITY;
    private final ElapsedTime timer;
    private final double a = 0.8; // a can be anything from 0 < a < 1
    private double reference;
    private double lastReference;
    private double error = 0;
    private double errorChange = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private double derivative = 0;
    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;

    public PID(double Kp, double Ki, double Kd, double reference) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = 0;
        this.integralSumLimit=0.25/Ki;
        this.lastReference = this.reference = reference;
        this.timer = new ElapsedTime();
    }

    public PID(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0);
    }

    public PID setReference(double reference) {
        this.reference = reference;
        return this;
    }

    public double getResult(double x) {
        // ref:https://www.ctrlaltftc.com/the-pid-controller/practical-improvements-to-pid

        // calculate the error
        error = x - reference;
        errorChange = (error - lastError);

        // filter out hight frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        derivative = currentFilterEstimate / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }

        // reset integral sum upon setpoint changes
        if (reference != lastReference) {
            integralSum = 0;
        }

        lastError = error;
        lastReference = reference;
        // reset the timer for next time
        timer.reset();
        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }

    public double getResult(double x, double reference) {
        this.reference = reference;
        return getResult(x);
    }

    public String toString() {
        return Misc.formatForUser("%s(p=%f i=%f d=%f f=%f)", getClass().getSimpleName(), Kp, Ki, Kd, Kf);
    }
}
