package org.firstinspires.ftc.teamcode.common.command;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;


@Config
public class SplineTracker {
    public static boolean DEBUG_RUN_FUNC = true;
    public static boolean DO_NOT_RUN_FUNC = false;
    public static boolean ASYNC_TASKS = false;   // true = 异步 fire-and-forget（旧行为）; false = 默认阻塞等待任务完成

    //region Dashboard
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    //endregion

    //region Point Details
    public double[] startPoint;
    public double lx, ly, ldx, ldy;
    public double x, y, dx, dy;
    public double heading, preH;
    public Runnable fn;

    // Tunable gains
    public static double K_HEADING = 0.02;     // 朝向P

    // Tangential velocity control
    public static double MAX_TANGENTIAL_VEL = 50.0;  // 机器能达到的最大速度(inch/s)
    public static int DERIVATIVE_SAMPLE_COUNT = 200;

    public static int SAMPLE_COUNT = 20;  // 曲线最近点计算取样数量
    public static int TIME_MAP_SAMPLES = 200;  // u→时间映射采样数

    // 加速度限制：对段内期望速度变化率（in/s²）。与段首、段尾速度约束结合，
    // 自动解算出当前沿样条的最大允许速度。设 INFINITY 或 <=0 时禁用限速。
    public static double MAX_ACCEL = 50;         // 切向加速度上限 (in/s²), 80≈0.2g

    public static int NEWTON_ITERS = 3;
    public static double END_U_THRESHOLD = 0.935;
    public static double END_POS_THRESHOLD = 1.0;
    public static double END_HEADING_THRESHOLD = 0.5;

    public static double TURN_DEAD_AREA = 0.0;
    public static double motorVoltage = 12;
    public static double VOLTAGE_COMP_WEIGHT = 0.0;

    // —— Look-ahead pursuit ——
    public static double LOOKAHEAD_MS = 15;       // n: prediction horizon in milliseconds

    // —— Position-error PID (shared gains for X and Y) ——
    public static double K_POS_P = 0.7;
    public static double K_POS_I = 0.6;
    public static double K_POS_D = 0.07;
    public static double POS_I_MAX = 0.5;
    public static double POS_D_ALPHA = 0.9;  // 滤波强度，[0,1]，越小滤波越强，但对变化敏感度越低
    public static double POS_I_WINDOW = 10.0;
    public static double POS_DEAD_ZONE = 0.0;

    public ElapsedTime runtime = new ElapsedTime();
    private final Robot robot;
    private double gotoX, gotoY, gotoH;

    // Velocity-profile state (per segment, retained from original)
    private double segmentMaxDerivative;
    private double segmentArcLength = 0;
    private double prevDesiredVel = 0;

    // Position PID state (per segment)
    private final PosPIDState posXState = new PosPIDState();
    private final PosPIDState posYState = new PosPIDState();
    private double prevTime = 0;
    private double segmentTotalTime = 0;   // 当前段预计算总时长 (s)
    private double[] timeMapCache;          // 当前段 u→t 映射
    private double[] velMapCache;           // 当前段 u→v 映射

    /**
     * Per-axis position-PID accumulator.
     */
    private static class PosPIDState {
        double integral = 0;
        double lastError = 0;
        double lastFilteredDeriv = 0;
        boolean firstRun = true;

        void reset() {
            integral = 0;
            lastError = 0;
            lastFilteredDeriv = 0;
            firstRun = true;
        }
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

    private double splineDerivative(double[] spline, double u) {
        return 3 * spline[0] * u * u + 2 * spline[1] * u + spline[2];
    }

    private double splineSecondDerivative(double[] spline, double u) {
        return 6 * spline[0] * u + 2 * spline[1];
    }

    private double clamp01(double u) {
        if (u < 0) return 0;
        if (u > 1) return 1;
        return u;
    }

    private double findClosestU(double[] splineX, double[] splineY, double rx, double ry) {
        // Step 1: Sample along the spline to find best initial u
        double bestU = 0;
        double bestDist = Double.MAX_VALUE;
        int samples = SAMPLE_COUNT;
        for (int i = 0; i <= samples; i++) {
            double u = (double) i / samples;
            double sx = spline_get(splineX, u);
            double sy = spline_get(splineY, u);
            double dx = sx - rx;
            double dy = sy - ry;
            double dist = dx * dx + dy * dy;
            if (dist < bestDist) {
                bestDist = dist;
                bestU = u;
            }
        }

        // Step 2: Newton refinement
        for (int iter = 0; iter < NEWTON_ITERS; iter++) {
            double u = bestU;
            double sx = spline_get(splineX, u);
            double sy = spline_get(splineY, u);
            double sdx = splineDerivative(splineX, u);
            double sdy = splineDerivative(splineY, u);
            double sddx = splineSecondDerivative(splineX, u);
            double sddy = splineSecondDerivative(splineY, u);

            double ex = sx - rx;
            double ey = sy - ry;

            // f'(u) = 2*(sx-rx)*sdx + 2*(sy-ry)*sdy
            double fprime = 2 * ex * sdx + 2 * ey * sdy;
            // f''(u) = 2*sdx*sdx + 2*ex*sddx + 2*sdy*sdy + 2*ey*sddy
            double fsecond = 2 * sdx * sdx + 2 * ex * sddx + 2 * sdy * sdy + 2 * ey * sddy;

            if (Math.abs(fsecond) < 1e-9) break;

            double uNew = u - fprime / fsecond;
            uNew = clamp01(uNew);

            if (Math.abs(uNew - u) < 1e-6) {
                bestU = uNew;
                break;
            }
            bestU = uNew;
        }

        return bestU;
    }

    private double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }

    /**
     * Single-axis position PID with EMA-filtered derivative, anti-windup integral,
     * and dead-zone handling. Returns a power command in [-1, 1].
     */
    private double computePositionPID(double error, double dt,
                                       double kp, double ki, double kd,
                                       double iWindow, double iMax,
                                       double dAlpha, double deadZone,
                                       PosPIDState state, double motorPowerGain) {
        if (state.firstRun) {
            state.firstRun = false;
            state.lastError = error;
            state.integral = 0;
            state.lastFilteredDeriv = 0;
            return kp * error * motorPowerGain;
        }

        if (Math.abs(error) < deadZone) {
            state.integral = 0;
            return 0;
        }

        // Proportional
        double pTerm = kp * error;

        // Integral with conditional anti-windup
        if (Math.abs(error) < iWindow) {
            state.integral += error * dt;
        } else {
            state.integral = 0;
        }
        state.integral = Math.max(-iMax, Math.min(iMax, state.integral));
        double iTerm = ki * state.integral;

        // Derivative with EMA filtering
        double rawDeriv = (error - state.lastError) / dt;
        double filteredDeriv = dAlpha * rawDeriv
                + (1 - dAlpha) * state.lastFilteredDeriv;
        double dTerm = kd * filteredDeriv;

        state.lastError = error;
        state.lastFilteredDeriv = filteredDeriv;

        return (pTerm + iTerm + dTerm) * motorPowerGain;
    }
    //endregion

    //region Initialization
    public SplineTracker(Robot robot) {
        this.robot = robot;
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odo.resetPosAndIMU();
    }

    public SplineTracker reset() {
        robot.odo.resetPosAndIMU();
        return this;
    }

    public SplineTracker setPose(Pose2D pose) {
        robot.odo.setGlobalPose(pose);
        return this;
    }
    //endregion

    //region Start Robot
    public SplineTracker startMove() {
        return this.startMove(0, 0, 0, 0, () -> {});
    }

    public SplineTracker startMove(Runnable fn) {
        return this.startMove(0, 0, 0, 0, fn);
    }

    public SplineTracker startMove(double x, double y) {
        return this.startMove(x, y, 0, 0, () -> {});
    }

    public SplineTracker startMove(double x, double y, Runnable fn) {
        return this.startMove(x, y, 0, 0, fn);
    }

    public SplineTracker startMove(double x, double y, double dx, double dy) {
        return this.startMove(x, y, dx, dy, () -> {});
    }

    public SplineTracker startMove(double[] point) {
        return this.startMove(point[0], point[1], point[2], point[3]);
    }

    public SplineTracker startMove(double x, double y, double dx, double dy, Runnable fn) {
        startPoint = new double[]{x, y, dx, dy};
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;
        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) {
            executeTask("startMove[0]", fn);
        }
        this.heading = getHeading();
        this.preH = getHeading();

        runtime.reset();

        packet.fieldOverlay()
                .drawImage("/dash/decode.png", 0, 0, 144, 144, Math.toRadians(-90), 72, 72, false);

        return this;
    }
    //endregion

    //region Control
    public SplineTracker stopMotor() {
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odoDrivetrain.stopMotor();
        return this;
    }

    //region Clock-driven u→t mapping (breaks zero-speed deadlock)
    /**
     * 预计算沿 spline 的速度曲线，生成 u→t / u→v 映射表。
     * 模拟加速度限制 + 减速度距离帽，数值积分得出段内任意时刻的目标 u。
     */
    private void precomputeTimeMap(double[] splineX, double[] splineY,
                                   double segMaxDeriv, double segTotalArc) {
        int N = TIME_MAP_SAMPLES;
        double[] cumArc = new double[N + 1];
        double[] timeMap = new double[N + 1];
        double[] velMap = new double[N + 1];

        // Pass 1: cumulative arc length
        cumArc[0] = 0;
        double prevSx = spline_get(splineX, 0);
        double prevSy = spline_get(splineY, 0);
        for (int i = 1; i <= N; i++) {
            double u = (double) i / N;
            double sx = spline_get(splineX, u);
            double sy = spline_get(splineY, u);
            cumArc[i] = cumArc[i - 1] + Math.hypot(sx - prevSx, sy - prevSy);
            prevSx = sx;
            prevSy = sy;
        }
        double totalArc = cumArc[N];

        // Pass 2: forward velocity-profile simulation
        timeMap[0] = 0;
        velMap[0] = 0;
        double prevV = 0;

        for (int i = 1; i <= N; i++) {
            double u = (double) i / N;

            // Derivative magnitude at this u
            double ds_du = Math.hypot(splineDerivative(splineX, u),
                                       splineDerivative(splineY, u));
            if (ds_du < 1e-6) {
                ds_du = segTotalArc;          // degenerate waypoint fallback
                if (ds_du < 1e-6) ds_du = 1.0;
            }

            // Raw target velocity from curvature ratio
            double rawTarget = (segMaxDeriv > 1e-6)
                    ? (ds_du / segMaxDeriv) * MAX_TANGENTIAL_VEL
                    : 0;

            // Deceleration distance cap
            double remainingArc = totalArc - cumArc[i];
            double maxVelDist = Math.sqrt(2.0 * MAX_ACCEL * remainingArc);
            double targetV = Math.min(rawTarget, maxVelDist);

            // Arc length of this u-step
            double ds = cumArc[i] - cumArc[i - 1];

            // Estimate dt from previous velocity, apply acceleration limit
            double vEst = Math.max(prevV, 0.001);
            double dtEst = ds / vEst;
            double maxDV = MAX_ACCEL * dtEst;
            double desiredV = prevV + Math.max(-maxDV, Math.min(maxDV, targetV - prevV));

            // Refine dt with average velocity
            double vAvg = (prevV + desiredV) / 2.0;
            if (vAvg < 0.001) vAvg = 0.001;
            double dt = ds / vAvg;

            // Re-apply accel limit with refined dt
            maxDV = MAX_ACCEL * dt;
            desiredV = prevV + Math.max(-maxDV, Math.min(maxDV, targetV - prevV));

            timeMap[i] = timeMap[i - 1] + dt;
            velMap[i] = desiredV;
            prevV = desiredV;
        }

        this.timeMapCache = timeMap;
        this.velMapCache = velMap;
        this.segmentTotalTime = timeMap[N];
    }

    /**
     * 二分查找 elapsed 时间对应的 u 值，线性插值。
     */
    private double lookupU(double elapsed) {
        double[] tm = this.timeMapCache;
        if (tm == null) return 0;
        int N = tm.length - 1;
        if (elapsed <= 0) return 0;
        if (elapsed >= tm[N]) return 1.0;

        int lo = 0, hi = N;
        while (lo < hi - 1) {
            int mid = (lo + hi) >>> 1;
            if (tm[mid] <= elapsed) lo = mid;
            else hi = mid;
        }

        double frac = (elapsed - tm[lo]) / (tm[hi] - tm[lo]);
        return (lo + frac) / N;
    }
    //endregion

    public SplineTracker addPoint(double x, double y, double dx, double dy) {
        return this.addPoint(x, y, dx, dy, () -> {});
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, Runnable fn) {
        return this.addPoint(x, y, dx, dy, this.heading, fn);
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, double heading) {
        return this.addPoint(x, y, dx, dy, heading, () -> {});
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, double heading, Runnable fn) {
        // Record last point (start of this spline segment)
        this.lx = this.x;
        this.ly = this.y;
        this.ldx = this.dx;
        this.ldy = this.dy;

        // Set target
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;

        // Build position splines
        double[] splineX = spline_fit(this.lx, this.ldx, this.x, this.dx);
        double[] splineY = spline_fit(this.ly, this.ldy, this.y, this.dy);

        // Unwrap target heading to shortest path from current heading
        double unwrappedHeading = heading;
        double hDiff = heading - this.heading;
        while (hDiff > 180) { unwrappedHeading -= 360; hDiff -= 360; }
        while (hDiff <= -180) { unwrappedHeading += 360; hDiff += 360; }

        double[] splineH = spline_fit(this.heading, 0, unwrappedHeading, 0);

        // Precompute segment max derivative magnitude for velocity normalization
        segmentMaxDerivative = 0;
        for (int i = 0; i <= DERIVATIVE_SAMPLE_COUNT; i++) {
            double u = (double) i / DERIVATIVE_SAMPLE_COUNT;
            double sdx = splineDerivative(splineX, u);
            double sdy = splineDerivative(splineY, u);
            double mag = Math.hypot(sdx, sdy);
            if (mag > segmentMaxDerivative) segmentMaxDerivative = mag;
        }

        // Pre-compute segment arc length for deceleration distance cap
        {
            double prevSx = spline_get(splineX, 0);
            double prevSy = spline_get(splineY, 0);
            segmentArcLength = 0;
            for (int i = 1; i <= DERIVATIVE_SAMPLE_COUNT; i++) {
                double u = (double) i / DERIVATIVE_SAMPLE_COUNT;
                double sx = spline_get(splineX, u);
                double sy = spline_get(splineY, u);
                segmentArcLength += Math.hypot(sx - prevSx, sy - prevSy);
                prevSx = sx; prevSy = sy;
            }
        }

        // Pre-compute clock-driven u→t mapping (breaks zero-speed deadlock)
        precomputeTimeMap(splineX, splineY, segmentMaxDerivative, segmentArcLength);

        // Reset velocity-profile and position-PID state for this segment
        prevDesiredVel = 0;
        prevTime = 0;
        posXState.reset();
        posYState.reset();

        double segmentStartTime = runtime.seconds();

        while (robot.opMode.opModeIsActive()) {
            robot.odo.update();

            double curH = getHeading();
            double rx = getX();
            double ry = getY();

            double rawMotorPowerGain = Math.abs(motorVoltage / robot.getVoltage());
            double motorPowerGain = 1.0 + (rawMotorPowerGain - 1.0) * VOLTAGE_COMP_WEIGHT;

            // ── Find closest point on spline (for exit condition & telemetry) ──
            double uStar = findClosestU(splineX, splineY, rx, ry);

            // ── Clock-driven u_target: time-based + look-ahead window ──
            double elapsed = runtime.seconds() - segmentStartTime;
            double uTarget = (segmentTotalTime < 0.001 || elapsed >= segmentTotalTime)
                    ? 1.0
                    : lookupU(elapsed + LOOKAHEAD_MS / 1000.0);

            // ── Velocity profile at uStar (telemetry / monitoring only) ──
            double currDerivMag = Math.hypot(splineDerivative(splineX, uStar),
                                              splineDerivative(splineY, uStar));
            if (currDerivMag < 1e-6) {
                currDerivMag = Math.hypot(this.x - this.lx, this.y - this.ly);
                if (currDerivMag < 1e-6) currDerivMag = 1.0;
            }
            double rawTargetVel = (segmentMaxDerivative > 1e-6)
                    ? (currDerivMag / segmentMaxDerivative) * MAX_TANGENTIAL_VEL
                    : 0;

            // Time step for position PID
            double currentTime = System.nanoTime() / 1e9;
            double dt = currentTime - prevTime;
            if (dt <= 0 || dt > 0.5) dt = 0.02;
            prevTime = currentTime;

            // Velocity profile at uStar (monitoring: what speed the robot's position calls for)
            double remainingArc = segmentArcLength * (1.0 - uStar);
            double maxVelFromDist = Math.sqrt(2.0 * MAX_ACCEL * remainingArc);
            double targetVel = Math.min(rawTargetVel, maxVelFromDist);
            double maxDeltaV = MAX_ACCEL * dt;
            double desiredVel = prevDesiredVel
                    + Math.max(-maxDeltaV, Math.min(maxDeltaV, targetVel - prevDesiredVel));
            prevDesiredVel = desiredVel;

            // Expected position at look-ahead point
            double targetX = spline_get(splineX, uTarget);
            double targetY = spline_get(splineY, uTarget);
            double targetH = spline_get(splineH, uTarget);

            // ── New: position-error PID (X and Y axes independently) ──
            double ex = targetX - rx;
            double ey = targetY - ry;
            double vx = computePositionPID(ex, dt, K_POS_P, K_POS_I, K_POS_D,
                    POS_I_WINDOW, POS_I_MAX, POS_D_ALPHA, POS_DEAD_ZONE,
                    posXState, motorPowerGain);
            double vy = computePositionPID(ey, dt, K_POS_P, K_POS_I, K_POS_D,
                    POS_I_WINDOW, POS_I_MAX, POS_D_ALPHA, POS_DEAD_ZONE,
                    posYState, motorPowerGain);

            // Heading control (unchanged)
            double headingError = angleDiff(targetH, curH);
            double yawPower = headingError * K_HEADING * motorPowerGain;
            if (Math.abs(headingError) < TURN_DEAD_AREA) yawPower = 0;

            vx = Range.clip(vx, -1, 1);
            vy = Range.clip(vy, -1, 1);
            yawPower = Range.clip(yawPower, -1, 1);

            // Drive robot
            if (!Globals.DEBUG)
                robot.odoDrivetrain.driveRobotFieldCentric(
                        vx, -vy, -yawPower
                );

            // FTC Dashboard
            if (Globals.DEBUG) robot.sleep(200);

            // Telemetry
            robot.telemetry.addData("u*", uStar);
            robot.telemetry.addData("uTarget", uTarget);
            robot.telemetry.addData("targetX", targetX);
            robot.telemetry.addData("targetY", targetY);
            robot.telemetry.addData("desiredH", targetH);
            robot.telemetry.addData("errorX", ex);
            robot.telemetry.addData("errorY", ey);
            robot.telemetry.addData("vxCmd", vx);
            robot.telemetry.addData("vyCmd", vy);
            robot.telemetry.addData("desiredVel", desiredVel);
            robot.telemetry.addData("elapsed", String.format("%.2f", elapsed));
            robot.telemetry.addData("segTime", String.format("%.2f", segmentTotalTime));
            if (velMapCache != null && uTarget < 1.0)
                robot.telemetry.addData("clockVel", String.format("%.1f",
                        velMapCache[Math.min((int)(uTarget * TIME_MAP_SAMPLES), TIME_MAP_SAMPLES)]));
            robot.telemetry.addLine();

            robot.telemetry.addData("X_POS", rx);
            robot.telemetry.addData("Y_POS", ry);
            robot.telemetry.addData("Heading", curH);
            robot.telemetry.addData("Voltage", robot.getVoltage());
            robot.telemetry.addData("MOTOR_GAIN", motorPowerGain);
            robot.telemetry.addData("Odo", robot.odo.getPosition().toString());

            Log.i("SplineTracker", "[XPow] " + vx);
            Log.i("SplineTracker", "[YPow] " + vy);
            Log.i("SplineTracker", "[ZPow] " + -yawPower);
            Log.i("SplineTracker", "[uTarget] " + uTarget);
            Log.i("SplineTracker", "[desiredVel] " + desiredVel + " in/s");
            Log.i("SplineTracker", "[elapsed] " + String.format("%.2f", elapsed) + "/" + String.format("%.2f", segmentTotalTime) + "s");

            // Store for accessors
            gotoX = targetX;
            gotoY = targetY;
            gotoH = targetH;

            // Exit condition: reached end of path segment
            double endPosError = Math.hypot(this.x - rx, this.y - ry);
            boolean done = uStar >= END_U_THRESHOLD
                    && endPosError < END_POS_THRESHOLD
                    && Math.abs(headingError) < END_HEADING_THRESHOLD;
            if (done) {
                // Zero-vel endpoint: actively brake so the robot doesn't coast
                boolean endVelIsZero = Math.abs(this.dx) < 0.1 && Math.abs(this.dy) < 0.1;
                if (endVelIsZero) stopMotor();
                break;
            }
        }

        this.heading = unwrappedHeading;
        this.preH = unwrappedHeading;

        // Run function at end of segment
        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) {
            executeTask("addPoint:segment-end", fn);
        } else {
            Log.w("SplineTracker", "[BRK4] addPoint: task SKIPPED — DEBUG=" + Globals.DEBUG
                    + ", DEBUG_RUN_FUNC=" + DEBUG_RUN_FUNC + ", DO_NOT_RUN_FUNC=" + DO_NOT_RUN_FUNC);
        }

        return this;
    }

    public SplineTracker addVelocity(double dx, double dy) {
        this.ldx = this.dx;
        this.ldy = this.dy;
        this.dx = dx;
        this.dy = dy;
        return this;
    }

    public SplineTracker addFunc(Runnable fn) {
        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) {
            executeTask("addFunc", fn);
        } else {
            Log.w("SplineTracker", "[BRK4] addFunc: task SKIPPED — DEBUG=" + Globals.DEBUG
                    + ", DEBUG_RUN_FUNC=" + DEBUG_RUN_FUNC + ", DO_NOT_RUN_FUNC=" + DO_NOT_RUN_FUNC);
        }
        return this;
    }

    /**
     * 执行 task，默认阻塞（{@link #ASYNC_TASKS}=false）。
     * 设为 true 时恢复旧的异步 fire-and-forget 行为。
     */
    private void executeTask(String location, Runnable fn) {
        if (ASYNC_TASKS) {
            Log.d("SplineTracker", "[BRK4-ASYNC] " + location + ": fire-and-forget");
            TaskLoopFrame.runOnce(fn);
        } else {
            Log.d("SplineTracker", "[BRK4-BLOCK] " + location + ": start");
            try {
                fn.run();
                Log.d("SplineTracker", "[BRK4-BLOCK] " + location + ": done");
            } catch (Exception e) {
                Log.e("SplineTracker", "[BRK4-BLOCK] " + location + ": exception", e);
            }
        }
    }

    public SplineTracker addTime(double seconds) {
        return this.addTime(seconds, () -> {});
    }

    public SplineTracker addTime(double seconds, Runnable fn) {
        double startWait = runtime.seconds();

        while (runtime.seconds() - startWait < seconds && robot.opMode.opModeIsActive()) {
            robot.odo.update();

            double curH = getHeading();
            double headingError = angleDiff(this.heading, curH);
            double yawPower = headingError * K_HEADING;
            if (Math.abs(headingError) < TURN_DEAD_AREA) yawPower = 0;
            yawPower = Range.clip(yawPower, -1, 1);

            if (!Globals.DEBUG)
                robot.odoDrivetrain.driveRobotFieldCentric(0, 0, -yawPower);
            else
                robot.sleep(200);

            robot.telemetry.addData("addTime", seconds);
            robot.telemetry.addData("remaining", String.format("%.2f", seconds - (runtime.seconds() - startWait)));
            robot.telemetry.addLine();
        }

        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) {
            executeTask("addTime:hold-end", fn);
        } else {
            Log.w("SplineTracker", "[BRK4] addTime: task SKIPPED — DEBUG=" + Globals.DEBUG
                    + ", DEBUG_RUN_FUNC=" + DEBUG_RUN_FUNC + ", DO_NOT_RUN_FUNC=" + DO_NOT_RUN_FUNC);
        }

        return this;
    }

    public SplineTracker addHeading(double heading) {
        return this.addHeading(heading, () -> {});
    }

    public SplineTracker addHeading(double heading, Runnable fn) {
        return this.addPoint(this.x, this.y, this.dx, this.dy, heading, fn);
    }
    //endregion

    //region Get Data
    public double getX() {
        return this.getX(DistanceUnit.INCH);
    }

    public double getX(DistanceUnit unit) {
        return robot.odo.getPosition().getX(unit);
    }

    public double getY() {
        return this.getY(DistanceUnit.INCH);
    }

    public double getY(DistanceUnit unit) {
        return robot.odo.getPosition().getY(unit);
    }

    public double getHeading() {
        return this.getHeading(AngleUnit.DEGREES);
    }

    public double getHeading(AngleUnit unit) {
        return robot.odo.getPosition().getHeading(unit);
    }

    public Pose2D getPosition() {
        return robot.odo.getPosition();
    }

    public double[] getStartPoint() {
        return this.startPoint;
    }

    public double[] getGotoPoint() {
        return new double[]{this.x, this.y, this.dx, this.dy};
    }

    public double[] getCurrentGotoPoint() {
        return new double[]{gotoX, gotoY};
    }

    public double[] getPreviousPoint() {
        return new double[]{this.lx, this.ly, this.ldx, this.ldy};
    }
    //endregion

    public Robot getRobot() {
        return robot;
    }
}
