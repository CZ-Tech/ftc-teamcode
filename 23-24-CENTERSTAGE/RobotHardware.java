/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.opmode.route.Route;
import org.firstinspires.ftc.teamcode.utlity.FileUtil;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */
//@Config
public class RobotHardware {
    // Rev HD Hex Motor (No Gearbox) : 28 counts/revolution
    // eg: GoBILDA 312 RPM (19.2:1) Yellow Jacket 537.7=28*19.2
    // 40:1 Rev 28*40=1120
    // 20:1 Rev 28*20=560
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.00;     // 96cm For figuring circumference
    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    static final double P_STRAFE_GAIN = 0.025;   // Strafe Speed Control "Gain".
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
//    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "model_20231220_084721.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {"bb", "blue-beacon", "rb", "red-beacon",};
    /* Declare OpMode members. */
    public final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    private final Telemetry telemetry;
    public Route route;
    public Vision vision;
    public FileUtil file;
    public AprilTagProcessor aprilTag;
    public TfodProcessor tfod;
    public DcMotor armMotor = null;
    public DcMotor leftIntake = null;
    //    public DcMotor rightIntake = null;
//    public DcMotor oneTimeMotor = null;
    public ServoImplEx launchDroneServo = null;

    public int dpad_right, dpad_left, dpad_up, dpad_down, y, a, right_bumper, left_bumper;
    public VisionPortal myVisionPortal;
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx leftBackDrive = null;

    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx rightBackDrive = null;
    public ServoImplEx intakeItself = null;
    public ServoImplEx intakeFront = null;
    public ServoImplEx intakeBack = null;
    public ServoImplEx intakeArm = null;
    public ServoImplEx dooropener = null;
    public double driveSpeed = 0.3;
    public String alliance = "";
    public int mission = 0;
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    private IMU imu = null;      // Control/Expansion Hub IMU
    public DistanceSensor sensorDistance = null;
    private double headingError = 0;
    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double turnSpeed = 0;

    public static boolean IS_DEBUG = false;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
        telemetry = opmode.telemetry;
        route = new Route(this);
        vision = new Vision(this);
        file = new FileUtil();
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     * 0 1
     * 3 2
     */
    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "lfmotor");  // Control Hub Motor 0
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rfmotor"); // Control Hub Motor 1
        rightBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rrmotor");  // Control Hub Motor 2
        leftBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "lrmotor");   // Control Hub Motor 3
        intakeItself = myOpMode.hardwareMap.get(ServoImplEx.class, "it");
        dooropener = myOpMode.hardwareMap.get(ServoImplEx.class, "dooropener");// Control Hub Servo 1

        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm");              // Expansion Hub Motor 0
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake = myOpMode.hardwareMap.get(DcMotor.class, "leftIntake");     // Expansion Hub Motor 1

        launchDroneServo = myOpMode.hardwareMap.get(ServoImplEx.class, "lds");        // Expansion Hub Servo 0
        intakeFront = myOpMode.hardwareMap.get(ServoImplEx.class, "itf");       // Expansion Hub Servo 1
        intakeBack = myOpMode.hardwareMap.get(ServoImplEx.class, "itb");        // Expansion Hub Servo 2
        intakeArm = myOpMode.hardwareMap.get(ServoImplEx.class, "itarm");       // Expansion Hub Servo 3

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchDroneServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeItself.setPwmRange(new PwmControl.PwmRange(1000, 2000));
        intakeFront.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeBack.setPwmRange(new PwmControl.PwmRange(500, 2500));
        dooropener.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        if (oneTimeMotor != null)
//            oneTimeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * TODO:  EDIT these two lines to match YOUR mounting configuration.
         */
        //TODO:control hub的朝向记得要改
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        sensorDistance = myOpMode.hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
//        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
//        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
//        leftHand.setPosition(MID_SERVO);
//        rightHand.setPosition(MID_SERVO);

        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();
//        telemetry.speak("Hardware Initialized");
//        telemetry.speak("六", "zh", "CN");
    }


    // ########################################################################################
    // !!!                                    基础运动模块                                   !!!!!
    // ########################################################################################


    /**
     * 基于场地坐标系进行操作
     *
     * @param axial   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral Right/Left driving power (-1.0 to 1.0) +ve is left
     * @param yaw     Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobotFieldCentric(double axial, double lateral, double yaw) {
        double botHeading = getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
        driveRobot(rotY, rotX, yaw);
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral Right/Left driving power (-1.0 to 1.0) +ve is forward
     * @param yaw     Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        lateral *= 1.1;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double   denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double leftFrontPower = (axial + lateral + yaw) / denominator;
        double rightFrontPower = (axial - lateral - yaw) / denominator;
        double leftBackPower = (axial - lateral + yaw) / denominator;
        double rightBackPower = (axial + lateral - yaw) / denominator;

//        leftFrontPower  *=0.5 ;
//        rightFrontPower *=0.5 ;
//        leftBackPower   *=0.5 ;
//        rightBackPower  *=0.5 ;


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
        if (myOpMode.gamepad1.share && IS_DEBUG) {
            leftFrontPower = myOpMode.gamepad1.x ? 0.3 : 0;
            rightFrontPower = myOpMode.gamepad1.y ? 0.3 : 0;
            leftBackPower = myOpMode.gamepad1.a ? 0.3 : 0;
            rightBackPower = myOpMode.gamepad1.b ? 0.3 : 0;
        }

        // Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }


    /**
     * 单独设置每一个电机的功率。一般情况下无需单独控制单个电机。
     *
     * @param leftFrontPower  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontPower Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackPower   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackPower  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void setDrivePower(double power) {
        setDrivePower(power, power, power, power);
    }

    /**
     * Stop All drive motors.
     */
    public void stopMotor() {
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * 设置电机运行模式
     *
     * @param mode 设置运行模式
     */
    public void setRunMode(DcMotor.RunMode mode) {
        // 设置电机运行模式
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    /**
     * 设置电机零功率运行模式，用于刹车。有两种模式 BRAKE会使电机减速
     * FLOAT会使电机滑行到停止位置
     *
     * @param behavior 设置运行模式
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        // 设置电机运行模式
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * 设置电机setTargetPosition
     *
     * @param moveCounts .getCurrentPosition()+moveCounts
     */
    public void setTargetPosition(int moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + moveCounts);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + moveCounts);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + moveCounts);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + moveCounts);
    }

    /**
     * 设置电机setTargetPosition
     *
     * @param moveCounts .getCurrentPosition()+moveCounts
     */
    public void setStrafeTargetPosition(int moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + moveCounts);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - moveCounts);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - moveCounts);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + moveCounts);
    }

    /**
     * 设置电机setTargetPosition
     *
     * @param yCounts .getCurrentPosition()+moveCounts
     * @param xCounts .getCurrentPosition()+moveCounts
     */
    public void setTranslateTargetPosition(int yCounts, int xCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + yCounts + xCounts);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + yCounts - xCounts);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + yCounts - xCounts);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + yCounts + xCounts);
        telemetry.addData("counts", "y:%d x:%d", yCounts, xCounts);
        telemetry.addData("position", "%n%d%n%d%n%d%n%d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition()
        );
        telemetry.update();
    }

    /**
     * isAllBusy
     */
    public boolean isAllBusy() {
        return leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy();
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public RobotHardware setArmPower(double power) {
        armMotor.setPower(power);
        return this;
    }

    public RobotHardware setLeftIntakePower(double power) {
        leftIntake.setPower(power);
        return this;
    }

    public RobotHardware setRightIntakePower(double power) {
//        rightIntake.setPower(power);
        return this;
    }

    public RobotHardware setOneTimeMotorPower(double power) {
//        oneTimeMotor.setPower(power);
        return this;
    }

    public RobotHardware Launch(double position) {
        launchDroneServo.setPosition(position);
        return this;
    }

    public RobotHardware setIntakeFrontPosition(IntakeFrontPosition position) {
        intakeFront.setPosition(position.getPosition());
        return this;
    }

    public RobotHardware setIntakeBackPosition(IntakeBackPosition position) {
        intakeBack.setPosition(position.getPosition());
        return this;
    }

    public RobotHardware setIntakeArmPosition(IntakeArmPosition position) {
        intakeArm.setPosition(position.getPosition());
        return this;
    }


    public RobotHardware setIntakeRoller(IntakeRollerPosition position) {
        intakeItself.setPosition(position.getPosition());
        return this;
    }


    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
//        leftHand.setPosition(MID_SERVO + offset);
//        rightHand.setPosition(MID_SERVO - offset);
    }

    public void initVision(VisionProcessor... processor) {
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(1280, 720))
                .addProcessors(processor)
                .build();
    }

    // ########################################################################################
    // !!!                                    视觉模块                                      !!!!!
    // ########################################################################################
    public RobotHardware initAprilTagVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
//                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                .setLensIntrinsics(837.197285915, 837.197285915, 376.991011136, 270.084582119)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);
        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder().setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessors(aprilTag).build();
        } else {
            myVisionPortal = new VisionPortal.Builder().setCamera(BuiltinCameraDirection.BACK).addProcessors(aprilTag).build();
        }
        return this;
    }

    public void initTfodVision() {
        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
        tfod = new TfodProcessor.Builder()
                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();
        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder().setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessors(tfod).build();
        } else {
            myVisionPortal = new VisionPortal.Builder().setCamera(BuiltinCameraDirection.BACK).addProcessors(tfod).build();
        }
    }

    public void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);


        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()
                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder().setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessors(tfod, aprilTag).build();
        } else {
            myVisionPortal = new VisionPortal.Builder().setCamera(BuiltinCameraDirection.BACK).addProcessors(tfod, aprilTag).build();
        }
    }

    /**
     * closeVision
     */
    public void closeVision() {
        myVisionPortal.close();
    }

    public void setCameraMode(ExposureControl.Mode mode) {
        while (!myOpMode.isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            myOpMode.sleep(20);
        }
        myVisionPortal
                .getCameraControl(ExposureControl.class)
                .setMode(mode);
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (myVisionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!myOpMode.isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested()) {
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    /**
     * @param DESIRED_TAG_ID
     */
    public AprilTagDetection getAprilTag(int DESIRED_TAG_ID) {

        boolean targetFound = false;
        AprilTagDetection desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }

        return desiredTag;
    }

    public RobotHardware translateToAprilTag(AprilTagDetection desiredTag, double Delta_Y, double Delta_X, double heading) {
        double rangeError = (desiredTag.ftcPose.range - Delta_Y);
        double yawError = desiredTag.ftcPose.yaw;
        double headingError = -Math.tan(Math.toRadians(desiredTag.ftcPose.bearing - yawError)) * desiredTag.ftcPose.range + Delta_X;
        turnToHeading(heading, driveSpeed);
        driveStrafe(headingError, heading, driveSpeed);
        driveStraight(rangeError, heading, driveSpeed);
        return this;
    }

    public RobotHardware translateToAprilTag(AprilTagDetection desiredTag, HashMap<String, Double> args, String name) {
        return translateToAprilTag(desiredTag, args.get(name + ".delta_y"), args.get(name + ".delta_x"), args.get(name + ".heading"));
    }

    public RobotHardware driveToAprilTag(HashMap<String, Double> args, String name, int tag_id) {
        return driveToAprilTag(tag_id, args.get("delta_y"), args.get("delta_x"), args.get("heading"), args.get("drivespeed"));
    }


    /**
     * @param Tag_ID
     * @param Delta_Y
     * @param Delta_X
     * @param heading
     */


    public void driveToAprilTag(int Tag_ID, double Delta_Y, double Delta_X, double heading) {
        driveToAprilTag(Tag_ID, Delta_Y, Delta_X, heading, this.driveSpeed);
    }

    /**
     * @param Tag_ID
     * @param Delta_Y
     * @param Delta_X
     * @param heading
     * @param driveSpeed
     * @return
     */
    public RobotHardware driveToAprilTag(int Tag_ID, double Delta_Y, double Delta_X, double heading, double driveSpeed) {

        telemetry.addData("2", null);       //TODO:调试BUG专用
        telemetry.update();
        AprilTagDetection desiredTag = null;
        while (!myOpMode.isStopRequested() && desiredTag == null) {
            desiredTag = getAprilTag(Tag_ID);
            if (desiredTag != null) telemetry.addData("x", desiredTag.ftcPose.x);
            if (desiredTag != null) telemetry.addData("y", desiredTag.ftcPose.y);
            telemetry.update();
        }

//        while (!myOpMode.gamepad1.ps) {
//        }

        double yawError = desiredTag.ftcPose.yaw;
//        double rangeError = (Math.cos(Math.toRadians(desiredTag.ftcPose.bearing)) * desiredTag.ftcPose.range) * 0.84 - Delta_Y;
//        double xError = (-Math.sin(Math.toRadians(desiredTag.ftcPose.bearing)) * desiredTag.ftcPose.range) * 1.02 + Delta_X;
//        double rangeError = desiredTag.ftcPose.y * 0.835 - Delta_Y;
//        double xError = desiredTag.ftcPose.x * 1.02 + Delta_X;
        double rangeError = desiredTag.ftcPose.y - Delta_Y;
        double xError = desiredTag.ftcPose.x + Delta_X;
//        closeVision();
        telemetry.addData("x", "%f %f %f", rangeError, yawError, headingError);
        telemetry.update();
//        turnToHeading(driveSpeed, heading);
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
//        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > 2.0) && runtime.seconds() < 2.0) {
//
//            // Determine required steering to keep on heading
//            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//
//            // Clip the speed to the maximum permitted value.
//            turnSpeed = Range.clip(turnSpeed, -driveSpeed, driveSpeed);
//
//            // Pivot in place by applying the turning correction
//
//            setDrivePower(-turnSpeed * 0.9, turnSpeed * 0.9, -turnSpeed * 2, turnSpeed * 2);
//            telemetry.addData("x", "%4.2f, %4.2f, %4.2f, %4.2f", driveSpeed, turnSpeed, heading, getHeading());
//            telemetry.update();
//        }
        // Stop all motion;
        stopMotor();
//        to(driveSpeed,rangeError,xError,heading);
        driveStrafe(xError, heading, driveSpeed);
        driveStraight(rangeError, heading, driveSpeed);
//        telemetry.addData("3", null); //TODO:调试BUG专用
//        telemetry.update();
        return this;
    }

    public void moveToAprilTag(AprilTagDetection desiredTag, double Delta_Y, double Delta_X) {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError = (desiredTag.ftcPose.range - Delta_Y);
        double yawError = desiredTag.ftcPose.yaw;
        double headingError = Math.tan(Math.toRadians(desiredTag.ftcPose.bearing)) * desiredTag.ftcPose.range - Delta_X;
        double yawMove = 0;
        double rangeBase = 0.07; // 启动功率0.09
        double headingBase = 0.08; //启动功率0.11
        double yawBase = 0.06;
        double rangeMove = 0;
        if (Math.abs(rangeError) < 0.5) {
            rangeMove = 0;
        } else {
            if (rangeError > 0) {
                rangeMove = rangeError * 0.020 + rangeBase;
            } else if (rangeError < 0) {
                rangeMove = rangeError * 0.020 - rangeBase;
            }
        }
        double headingMove = 0;
        if (Math.abs(headingError) < 1) {
            headingMove = 0;
        } else {
            if (headingError > 0) {
                headingMove = -headingError * 0.03 - headingBase;
            } else if (headingError < 0) {
                headingMove = -headingError * 0.03 + headingBase;
            }
        }
        if (Math.abs(yawError) < 12) {
            yawMove = 0;
        } else {
            if (yawError > 0) {
                yawMove = -yawError * 0.008 - yawBase;
            } else if (yawError < 0) {
                yawMove = -yawError * 0.008 + yawBase;
            }
        }
        telemetry.addData("headingError", headingError);
        driveRobot(rangeMove, headingMove, 0);
//        driveRobot(rangeMove, headingMove, yawMove);
    }
//        }else{
//            driveRobot(0,0,yawMove);
//        }
//        driveRobot(
//                Math.abs(rangeError) < 1 ? 0 : rangeError * 0.02 + 0.12,
//                headingMove,
//                yawMove
//        );
//        //以下使用三角函数
//        double yawMove = 0;
//        if (Math.abs(yawError)<1){
//            yawMove = 0;
//        }
//        else{
//            yawMove = yawError*1.2;
//            resetYaw();
//            turnToHeading(0.2,yawMove);
////            if(yawError>0){
////                yawMove = -yawError*0.015-0.12;
////            }else if(yawError<0){
////                yawMove = -yawError*0.015+0.12;
////            }
//        }
////        driveRobot(0,0,yawMove);
//        double headingMove = 0;
//        headingMove = Math.sin(headingError)*rangeError;
//        resetYaw();
//        telemetry.addData("headingMove:",headingMove);
//        telemetry.addData("headingError:",headingError);
//        driveStrafe(0.3,headingMove,0);
//        sleep(10000);


    public Recognition getTfod(String desiredLabel) {
        Recognition desiredTfod = null;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            double x = recognition.getLeft();
            double y = recognition.getTop();
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (desiredLabel.equals(recognition.getLabel())) {
                desiredTfod = recognition;
                break;
            }
        }   // end for() loop

        return desiredTfod;
    }

    public void moveToTfod(Recognition desiredTfod, double DESIRED_WIDTH_IN_SCREEN) {
        double x = (desiredTfod.getLeft() + desiredTfod.getRight()) / 2;
        double y = (desiredTfod.getTop() + desiredTfod.getBottom()) / 2;
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError = (1 - desiredTfod.getWidth() / DESIRED_WIDTH_IN_SCREEN);
//        double  yawError        = desiredTag.ftcPose.yaw;
        double headingError = 320 - x;
        driveRobot(rangeError * P_DRIVE_GAIN, 0, headingError * P_TURN_GAIN);
    }
    // ########################################################################################
    // !!!                                    陀螺仪相关代码                                 !!!!!
    // ########################################################################################

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        return getHeading(AngleUnit.DEGREES);
    }

    /**
     * read the Robot heading directly from the IMU
     *
     * @param unit AngleUnit.DEGREES / AngleUnit.RADIANS
     */
    public double getHeading(AngleUnit unit) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//        telemetry.addData("xyz", "%.2f %.2f %.2f", velocity.xRotationRate, velocity.yRotationRate, velocity.zRotationRate);
//        telemetry.addData("Yaw/Pitch/Roll", orientation.toString());
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        return orientation.getYaw(unit);
    }

    /**
     * resetYaw
     */
    public RobotHardware resetYaw() {
        imu.resetYaw();
        targetHeading = 0;
        return this;
    }
    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************
    // ########################################################################################
    // !!!                                    自动阶段相关代码                                 !!!
    // ########################################################################################

    /**
     * ↑往前距离
     *
     * @param distance
     * @return
     */
    public RobotHardware w(double distance) {
        return driveStraight(distance, targetHeading, driveSpeed);
    }

    public RobotHardware w(double distance, double speed) {
        return driveStraight(distance, targetHeading, speed);
    }

    public RobotHardware w(double distance, double heading, double speed) {
        return driveStraight(distance, heading, speed);
    }


    /**
     * 往后距离
     *
     * @param distance
     * @return
     */
    public RobotHardware s(double distance) {
        return driveStraight(-distance, targetHeading, driveSpeed);
    }

    public RobotHardware s(double distance, double speed) {
        return driveStraight(-distance, targetHeading, speed);
    }

    public RobotHardware s(double distance, double heading, double speed) {
        return driveStraight(-distance, heading, speed);
    }

    @Deprecated
    public RobotHardware driveStraight(double distance) {
        return driveStraight(distance, 0, driveSpeed);
    }

    /**
     * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading  Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from the current robotHeading.
     * @param speed    MAX Speed for forward/rev motion (range 0 to +1.0) .
     */
    public RobotHardware driveStraight(double distance, double heading, double speed) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            speed = Math.abs(speed);
            driveRobot(speed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (myOpMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -speed, speed);
                // Apply the turning correction to the current driving speed.
                driveRobot(speed, 0, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public RobotHardware driveStraight(HashMap<String, Double> args, String name) {
        return driveStraight(args.get(name + ".distance"), args.get(name + ".heading"), args.get(name + ".speed"));
    }

    public void driveRobot_ZYC(double axial, double lateral, double yaw) {
//        telemetry.addData("Speed", "Vy %5.2f, Vx %5.2f, Vr %5.2f ", axial, lateral, yaw);
        lateral *= 1.1;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double leftFrontPower = (axial + lateral + yaw) / denominator;
        double rightFrontPower = (axial - lateral - yaw) / denominator;
        double leftBackPower = (axial - lateral + yaw) / denominator;
        double rightBackPower = (axial + lateral - yaw) / denominator;

        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }


    public RobotHardware dsAndTurn(double distance, double heading, double speed, double turn) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            speed = Math.abs(speed);
            driveRobot(speed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (myOpMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -speed, speed);
                // Apply the turning correction to the current driving speed.
                driveRobot(speed, 0, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }


    public RobotHardware rushY(double distance, double heading) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop

            double driveSpeed = 0;
            double tempSpeed = 0;
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            // keep looping while we are still active, and BOTH motors are running.
            while (myOpMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -1, 1);
                if (runtime.seconds() < 0.5) {
                    driveSpeed = runtime.seconds() * 2;
                } else {
                    driveSpeed = 1;
                }
                tempSpeed = Math.abs(moveCounts - leftFrontDrive.getCurrentPosition()) / COUNTS_PER_MOTOR_REV / 2;
                if (tempSpeed < 1) driveSpeed = tempSpeed;
                // Apply the turning correction to the current driving speed.
                driveRobot(driveSpeed, 0, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public RobotHardware rush(HashMap<String, Double> args, String name) {
        return rush(args.get(name + ".distance"), args.get(name + ".heading"));
    }

    public RobotHardware rush(double distance, double heading) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop

            double driveSpeed = 0;
            double tempSpeed = 0;
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            // keep looping while we are still active, and BOTH motors are running.
            while (myOpMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -1, 1);
                if (runtime.seconds() < 1) {
                    driveSpeed = runtime.seconds();
                } else {
                    driveSpeed = 1;
                }
                tempSpeed = Math.abs(moveCounts - leftFrontDrive.getCurrentPosition()) / COUNTS_PER_MOTOR_REV;
                if (tempSpeed < 1) driveSpeed = tempSpeed;
                // Apply the turning correction to the current driving speed.
                driveRobot(driveSpeed, 0, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    /**
     * @param distance
     * @return
     */
    public RobotHardware d(double distance) {
        return driveStrafe(distance, targetHeading, driveSpeed);
    }

    public RobotHardware d(double distance, double speed) {
        return driveStrafe(distance, targetHeading, speed);
    }

    public RobotHardware d(double distance, double heading, double speed) {
        return driveStrafe(distance, heading, speed);
    }


    /**
     * @param distance
     * @return
     */
    public RobotHardware a(double distance) {
        return driveStrafe(-distance, targetHeading, driveSpeed);
    }

    public RobotHardware a(double distance, double speed) {
        return driveStrafe(-distance, targetHeading, speed);
    }

    public RobotHardware a(double distance, double heading, double speed) {
        return driveStrafe(-distance, heading, speed);
    }

    @Deprecated
    public RobotHardware driveStrafe(double distance) {
        return driveStrafe(distance, 0, driveSpeed);
    }

    /**
     * @param distance
     * @param heading
     * @param speed
     * @return
     */
    public RobotHardware driveStrafe(double distance, double heading, double speed) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setStrafeTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            speed = Math.abs(speed);
            driveRobot(0, speed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (myOpMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -speed, speed);
                // Apply the turning correction to the current driving speed.
                driveRobot(0, speed, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public RobotHardware driveStrafe(HashMap<String, Double> args, String name) {
        return driveStrafe(args.get(name + ".distance"), args.get(name + ".heading"), args.get(name + ".speed"));
    }

    public RobotHardware rushX(double distance, double heading) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setStrafeTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            double driveSpeed = 0;
            double tempSpeed = 0;
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();

            // keep looping while we are still active, and BOTH motors are running.
            while (myOpMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -1, 1);
                if (runtime.seconds() < 1) {
                    driveSpeed = runtime.seconds();
                } else {
                    driveSpeed = 1;
                }
                tempSpeed = Math.abs(moveCounts - Math.abs(leftFrontDrive.getCurrentPosition())) / COUNTS_PER_MOTOR_REV;
                if (tempSpeed < 1) driveSpeed = tempSpeed;
                // Apply the turning correction to the current driving speed.
                driveRobot(0, driveSpeed, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    /**
     * @param y
     * @param x
     * @param heading
     * @return
     */
    public RobotHardware to(double y, double x, double heading) {
        return to(y, x, heading, driveSpeed);
    }

    /**
     * @param y
     * @param x
     * @param heading
     * @param v
     * @return
     */
    public RobotHardware to(double y, double x, double heading, double v) {

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Determine new target position, and pass to motor controller
            int lfp = (int) ((y + x) * COUNTS_PER_INCH);
            int rfp = (int) ((y - x) * COUNTS_PER_INCH);
            int rbp = (int) ((y + x) * COUNTS_PER_INCH);
            int lbp = (int) ((y - x) * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(lfp);
            rightFrontDrive.setTargetPosition(rfp);
            leftBackDrive.setTargetPosition(lbp);
            rightBackDrive.setTargetPosition(rbp);

            double n = Math.sqrt(x * x + y * y);


            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            v = Math.abs(v);
            // Start driving straight, and then enter the control loop
            driveRobotFieldCentric(v * y / n, v * x / n, 0);
            lfp = Math.abs(lfp);
            rfp = Math.abs(rfp);
            lbp = Math.abs(lbp);
            rbp = Math.abs(rbp);

            int maxp = Math.max(lfp, Math.max(rfp, Math.max(rbp, lbp)));

            DcMotorEx busyMotor = leftFrontDrive;
            if (maxp == rfp) busyMotor = rightFrontDrive;
            if (maxp == lbp) busyMotor = leftBackDrive;
            if (maxp == rbp) busyMotor = rightBackDrive;


            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);


            // keep looping while we are still active, and BOTH motors are running.
            while (myOpMode.opModeIsActive() && busyMotor.isBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (y < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -v, v);

                // Apply the turning correction to the current driving speed.
                driveRobotFieldCentric(v * y / n, v * x / n, -turnSpeed);
                telemetry.addData("左前，右前，左后，右后", "%n %d%n  %d%n  %d%n  %d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(2000);
        }
        return this;
    }

    // ########################################################################################
    // !!!                                    WIP                                           !!!
    // ########################################################################################

    public void turnHead() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    /**
     * 自转相对角度，逆时针为正
     *
     * @param h 角度朝向
     * @return
     */
    public RobotHardware t(double h) {
        return turnToHeading(targetHeading + h, driveSpeed);
    }

    /**
     * 转到绝对值角度,逆时针为正
     *
     * @param h 角度朝向
     * @return
     */
    public RobotHardware G(double h) {
        return turnToHeading(h, driveSpeed);
    }

    /**
     * @param heading
     * @return
     */
    public RobotHardware turnToHeading(double heading) {
        return turnToHeading(heading, driveSpeed);
    }

    /**
     * @param heading
     * @param speed
     * @return
     */
    public RobotHardware turnToHeading(double heading, double speed) {
        return turnToHeading(heading, speed, 5);
    }

    /**
     * @param heading
     * @param speed
     * @param timeout
     * @return
     */
    public RobotHardware turnToHeading(double heading, double speed, double timeout) {
        return turnToHeading(heading, speed, timeout, P_DRIVE_GAIN);
    }

    public RobotHardware turnToHeading(HashMap<String, Double> args, String name) {
        return turnToHeading(args.get(name + ".heading"), args.get(name + ".speed"), args.get(name + "timeout"), P_DRIVE_GAIN);
    }

    /**
     * Spin on the central axis to point in a new direction.
     * <p>
     * Move will stop if either of these conditions occur:
     * <p>
     * 1) Move gets to the heading (angle)
     * <p>
     * 2) Driver stops the OpMode running.
     *
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                If a relative angle is required, add/subtract from current heading.
     * @param speed   Desired MAX speed of turn. (range 0 to +1.0)
     */
    public RobotHardware turnToHeading(double heading, double speed, double timeout, double P_DRIVE_GAIN) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (myOpMode.opModeIsActive()
                && (Math.abs(headingError) > HEADING_THRESHOLD)
                && Math.abs(myOpMode.gamepad1.right_stick_x) < 0.1
                && !myOpMode.gamepad1.dpad_down
                && !myOpMode.gamepad1.dpad_up
                && runtime.seconds() < timeout
        ) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            // Pivot in place by applying the turning correction
            driveRobot(0, 0, -turnSpeed);
//            telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f",maxTurnSpeed,turnSpeed,heading,getHeading());
//            telemetry.update();
        }

        // Stop all motion;
        stopMotor();
        return this;
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param heading  Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param speed    Maximum differential turn speed (range 0 to +1.0)
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public RobotHardware holdHeading(double heading, double speed, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            // Pivot in place by applying the turning correction
            driveRobot(0, 0, -turnSpeed);

        }

        // Stop all motion;
        stopMotor();
        return this;
    }

    public RobotHardware holdHeading(HashMap<String, Double> args, String name) {
        return holdHeading(args.get(name + ".heading"), args.get(name + ".speed"), args.get(name + ".holdtime"));
    }


    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * @param milliseconds
     * @return
     */
    public RobotHardware zzz(long milliseconds) {
        myOpMode.sleep(milliseconds);
        return this;
    }

    /**
     * @param milliseconds
     * @return
     */
    public RobotHardware sleep(long milliseconds) {
        myOpMode.sleep(milliseconds);
        return this;
    }

    /**
     *
     */
    public void updateButtonState() {
        if (myOpMode.gamepad1.dpad_up) {
            ++dpad_up;
        } else {
            dpad_up = 0;
        }
        if (myOpMode.gamepad1.dpad_down) {
            ++dpad_down;
        } else {
            dpad_down = 0;
        }
        if (myOpMode.gamepad1.dpad_left) {
            ++dpad_left;
        } else {
            dpad_left = 0;
        }
        if (myOpMode.gamepad1.dpad_right) {
            ++dpad_right;
        } else {
            dpad_right = 0;
        }
        if (myOpMode.gamepad1.y) {
            ++y;
        } else {
            y = 0;
        }
        if (myOpMode.gamepad1.a) {
            ++a;
        } else {
            a = 0;
        }
        if (myOpMode.gamepad1.right_bumper) {
            ++right_bumper;
        } else {
            right_bumper = 0;
        }
        if (myOpMode.gamepad1.left_bumper) {
            ++left_bumper;
        } else {
            left_bumper = 0;
        }


    }

    public boolean dpad_up_once() {
        return dpad_up == 1;
    }

    public boolean dpad_down_once() {
        return dpad_down == 1;
    }

    public boolean dpad_left_once() {
        return dpad_left == 1;
    }

    public boolean dpad_right_once() {
        return dpad_right == 1;
    }

    public boolean a_once() {
        return a == 1;
    }

    public boolean y_once() {
        return y == 1;
    }

    public boolean right_bumper_once() {
        return right_bumper == 1;
    }

    public boolean left_bumper_once() {
        return left_bumper == 1;
    }


    public RobotHardware setServoPosition(Servo servo, double targetPosition) {
        double currentPosition = servo.getPosition();

        if (Math.abs(currentPosition - targetPosition) < 0.005) return this;

        if (currentPosition < targetPosition) {
            servo.setPosition(currentPosition + 0.005);
        } else if (currentPosition > targetPosition) {
            servo.setPosition(currentPosition - 0.005);
        }

        return this;
    }

    public Double getargs(ArrayList<String[]> args, String key) {
        for (String[] i : args) {
            if (Objects.equals(i[0], key)) {
                return Double.parseDouble(i[1]);
            }
        }
        telemetry.addData("fail!", null);
        telemetry.update();
        sleep(1000);
        return 0.0;
    }

    public RobotHardware syncRun(Runnable... functions) {
        for (Runnable function : functions) {
            new Thread(function).start();
        }
        return this;
    }
    // Computes the current battery voltage
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public double getDynamicBatterySagCompensation(){
        return 12/getBatteryVoltage();
    }
}
