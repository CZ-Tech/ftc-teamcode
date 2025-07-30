package org.firstinspires.ftc.teamcode.common.drive;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Robot;

//@Confg
public class OdoDrivetrain {
    private final Robot robot;

    public DcMotorEx driveLeftFront = null;
    public DcMotorEx driveLeftBack = null;
    public DcMotorEx driveRightFront = null;
    public DcMotorEx driveRightBack = null;
    public DcMotorEx encoderLeft = null;
    public DcMotorEx encoderRight = null;
    public DcMotorEx encoderCenter = null;
    private double targetHeading = 0;
    private double turnSpeed = 0;
    private double headingError = 0;
    public static double P_DRIVE_STRAIGHT_GAIN = 0.002;
    public static double P_DRIVE_STRAFE_GAIN = 0.001;
    public double angleOffset = 0;

    public OdoDrivetrain(Robot robot) {
        this.robot = robot;
        // lfmotor 0 \______/ 1 rfmotor
        //            |    |
        //            |    |
        // lrmotor 3 /______\ 2 rrmotor

        driveLeftFront = robot.hardwareMap.get(DcMotorEx.class, "lfmotor");  // Control Hub Motor 0
        driveRightFront = robot.hardwareMap.get(DcMotorEx.class, "rfmotor"); // Control Hub Motor 1
        driveRightBack = robot.hardwareMap.get(DcMotorEx.class, "rrmotor");  // Control Hub Motor 2
        driveLeftBack = robot.hardwareMap.get(DcMotorEx.class, "lrmotor");   // Control Hub Motor 3

        driveLeftFront.setDirection(DcMotor.Direction.REVERSE); //forward
        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE); //reverse
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);


        encoderCenter = driveLeftFront; // robot.hardwareMap.get(DcMotorEx.class, "lfmotor");
        encoderLeft = driveRightFront; // robot.hardwareMap.get(DcMotorEx.class, "rfmotor");
        encoderRight = driveRightBack; // robot.hardwareMap.get(DcMotorEx.class, "rrmotor");

        encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getCenterPosition() {
        return encoderCenter.getCurrentPosition();
    }

    public double getLeftPosition() {
        return -encoderLeft.getCurrentPosition();
    }

    public double getRightPosition() {
        return encoderRight.getCurrentPosition();
    }

    public static double VOLTAGE = 12;

    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double powerLeftFront = (axial + lateral + yaw) / denominator;
        double powerRightFront = (axial - lateral - yaw) / denominator;
        double powerLeftBack = (axial - lateral + yaw) / denominator;
        double powerRightBack = (axial + lateral - yaw) / denominator;


        // TODO: 电机测试代码，反注释下方代码进行测试。
        // 一号手柄按住share键或者back键测试
        /**
         * Xbox/PS4 Button - Motor
         *   X / ▢         - Front Left
         *   Y / Δ         - Front Right
         *   B / O         - Rear  Right
         *   A / X         - Rear  Left
         *                                    The buttons are mapped to match the wheels spatially if you
         *                                    were to rotate the gamepad 45deg°. x/square is the front left
         *                    ________        and each button corresponds to the wheel as you go clockwise
         *                   / ______ \
         *     ------------.-'   _  '-..+              Front of Bot
         *              /   _  ( Y )  _  \                  ^
         *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
         *        ___  '.      ( A )     /|       Wheel       \      Wheel
         *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
         *     |       |                 |                      \
         *      '.___.' '.               |          Rear Left    \   Rear Right
         *               '.             /             Wheel       \    Wheel
         *                \.          .'              (A/X)        \   (B/O)
         *                  \________/
         *  https://rr.brott.dev/docs/v1-0/tuning/
         */
//        if (robot.opMode.gamepad1.share) {
//            powerLeftFront = robot.opMode.gamepad1.x ? 0.3 : 0;
//            powerRightFront = robot.opMode.gamepad1.y ? 0.3 : 0;
//            powerLeftBack = robot.opMode.gamepad1.a ? 0.3 : 0;
//            powerRightBack = robot.opMode.gamepad1.b ? 0.3 : 0;
//        }

        // Use existing function to drive both wheels.
        robot.telemetry.addData("LF", powerLeftFront);
        robot.telemetry.addData("RF", powerRightFront);
        robot.telemetry.addData("LB", powerLeftBack);
        robot.telemetry.addData("LR", powerRightBack);

        double MOTOR_GAIN = Math.abs(VOLTAGE / robot.getVoltage());
        setDrivePower(powerLeftFront, powerRightFront, powerLeftBack, powerRightBack);
    }

    public double getHeading(AngleUnit angleUnit) {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        AngularVelocity velocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return orientation.getYaw(angleUnit)+angleOffset;
    }

    public void resetYaw() {
        robot.imu.resetYaw();
    }

    public void driveRobotFieldCentric(double axial, double lateral, double yaw) {
        double botHeading = getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
        driveRobot(rotY, rotX, yaw);
    }

    public void setDrivePower(double power) {
        setDrivePower(power, power, power, power);
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        driveLeftFront.setPower(leftFrontPower);
        driveRightFront.setPower(rightFrontPower);
        driveLeftBack.setPower(leftBackPower);
        driveRightBack.setPower(rightBackPower);
    }

    public void stopMotor() {
        setDrivePower(0, 0, 0, 0);
    }

    public void setRunMode(DcMotor.RunMode mode) {
        driveLeftFront.setMode(mode);
        driveRightFront.setMode(mode);
        driveLeftBack.setMode(mode);
        driveRightBack.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveLeftFront.setZeroPowerBehavior(behavior);
        driveRightFront.setZeroPowerBehavior(behavior);
        driveLeftBack.setZeroPowerBehavior(behavior);
        driveRightBack.setZeroPowerBehavior(behavior);
    }

    public OdoDrivetrain driveStraight(double distance, double heading, double speed) {
        return driveStraight(distance, speed);
    }

    public OdoDrivetrain driveStraight(double distance, double speed) {
        encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the OpMode is still active
        if (robot.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            long moveCounts = (long) (distance / 1.88976378 / Math.PI * 2000);
            long targetPosition = encoderLeft.getCurrentPosition() + moveCounts;
            boolean finished = false;
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double driveSpeed = 0;
            double tempSpeed = 0;

            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();

            // keep looping while we are still active, and BOTH motors are running.
            while (robot.opMode.opModeIsActive() && !finished) {
                if (distance > 0) {
                    if (encoderLeft.getCurrentPosition() > targetPosition) finished = true;
                } else {
                    if (encoderLeft.getCurrentPosition() < targetPosition) finished = true;
                }
                turnSpeed = Range.clip((encoderRight.getCurrentPosition() - encoderLeft.getCurrentPosition())
                                * speed * P_DRIVE_STRAIGHT_GAIN,
                        -1, 1);
                if (runtime.seconds() < Math.abs(speed)) {
                    driveSpeed = runtime.seconds();
                } else {
                    driveSpeed = speed;
                }
                tempSpeed = Math.abs(targetPosition - encoderLeft.getCurrentPosition()) / 8000.0 + 0.2;
                if (tempSpeed < Math.abs(speed)) driveSpeed = tempSpeed;
                if (distance < 0) driveSpeed *= -1.0;
                driveRobot(driveSpeed * 12 / robot.getVoltage(), 0, turnSpeed);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            robot.telemetry.addData("driveSpeed", 0);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public OdoDrivetrain driveStrafe(double distance, double heading, double speed) {
        return driveStrafe(distance, speed);
    }

    public OdoDrivetrain driveStrafe(double distance, double speed) {
        encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Ensure that the OpMode is still active
        if (robot.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            long moveCounts = (long) (distance * 2000 / 1.88976378 / Math.PI);
            long targetPosition = encoderCenter.getCurrentPosition() + moveCounts;
            boolean finished = false;
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double driveSpeed = 0;
            double tempSpeed = 0;

            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();

            // keep looping while we are still active, and BOTH motors are running.
            while (robot.opMode.opModeIsActive() && !finished) {
                if (distance > 0) {
                    if (encoderCenter.getCurrentPosition() > targetPosition) finished = true;
                } else {
                    if (encoderCenter.getCurrentPosition() < targetPosition) finished = true;
                }
                // Determine required steering to keep on heading
                turnSpeed = Range.clip((encoderRight.getCurrentPosition() - encoderLeft.getCurrentPosition()) * speed * P_DRIVE_STRAFE_GAIN, -1, 1);

                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -1, 1);
                if (runtime.seconds() < Math.abs(speed)) {
                    driveSpeed = runtime.seconds();
                } else {
                    driveSpeed = speed;
                }
                tempSpeed = Math.abs(targetPosition - encoderCenter.getCurrentPosition()) / 8000.0 + 0.2;
                if (tempSpeed < Math.abs(speed)) driveSpeed = tempSpeed;
                if (distance < 0) driveSpeed *= -1.0;
                // Apply the turning correction to the current driving speed.
                driveRobot(0, driveSpeed * 12 / robot.getVoltage(), turnSpeed);

                robot.telemetry.addData("runtime", runtime.time());
                robot.telemetry.addData("tempSpeed", tempSpeed);
                robot.telemetry.addData("driveSpeed", driveSpeed);
                robot.telemetry.addData("moveCounts", moveCounts);
                robot.telemetry.addData("targetPosition", targetPosition);
                robot.telemetry.addData("encoderCenter", encoderCenter.getCurrentPosition());
                robot.telemetry.addData("encoderLeft", encoderLeft.getCurrentPosition());
                robot.telemetry.addData("encoderRight", encoderRight.getCurrentPosition());
                robot.telemetry.addData("turnSpeed", turnSpeed);
//                robot.telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            robot.telemetry.addData("driveSpeed", 0);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public OdoDrivetrain driveStraighfe(double distance, double angle, double speed) {
        encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the OpMode is still active
        if (robot.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            long moveCounts = (long) (distance * 2000 / 1.88976378 / Math.PI);
            long targetPosition = encoderCenter.getCurrentPosition() + moveCounts;
            boolean finished = false;
            setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double driveSpeed = 0;
            double tempSpeed = 0;

            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();

            // keep looping while we are still active, and BOTH motors are running.
            while (robot.opMode.opModeIsActive() && !finished) {
                if (distance > 0) {
                    if (encoderCenter.getCurrentPosition() > targetPosition) finished = true;
                } else {
                    if (encoderCenter.getCurrentPosition() < targetPosition) finished = true;
                }
                // Determine required steering to keep on heading
                turnSpeed = Range.clip((encoderRight.getCurrentPosition() - encoderLeft.getCurrentPosition()) * speed * P_DRIVE_STRAFE_GAIN, -1, 1);

                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -1, 1);
                if (runtime.seconds() < Math.abs(speed)) {
                    driveSpeed = runtime.seconds();
                } else {
                    driveSpeed = speed;
                }
                tempSpeed = Math.abs(targetPosition - encoderCenter.getCurrentPosition()) / 8000.0 + 0.2;
                if (tempSpeed < Math.abs(speed)) driveSpeed = tempSpeed;
                if (distance < 0) driveSpeed *= -1.0;
                // Apply the turning correction to the current driving speed.
                double finalSpeed = driveSpeed * 12 / robot.getVoltage();
                driveRobot(finalSpeed, finalSpeed / Math.tan(Math.toRadians(angle)), 0);


                robot.telemetry.addData("runtime", runtime.time());
                robot.telemetry.addData("tempSpeed", tempSpeed);
                robot.telemetry.addData("driveSpeed", driveSpeed);
                robot.telemetry.addData("moveCounts", moveCounts);
                robot.telemetry.addData("targetPosition", targetPosition);
                robot.telemetry.addData("encoderCenter", encoderCenter.getCurrentPosition());
                robot.telemetry.addData("encoderLeft", encoderLeft.getCurrentPosition());
                robot.telemetry.addData("encoderRight", encoderRight.getCurrentPosition());
                robot.telemetry.addData("turnSpeed", turnSpeed);
//                robot.telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            robot.telemetry.addData("driveSpeed", 0);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    private boolean isAllBusy() {
        return driveLeftFront.isBusy() && driveLeftBack.isBusy() && driveRightFront.isBusy() && driveRightBack.isBusy();
    }

    public OdoDrivetrain sleep(long ms) {
        robot.sleep(ms);
        return this;
    }
}
