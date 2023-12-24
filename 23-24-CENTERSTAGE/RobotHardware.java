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

 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.IMU;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;
 
 import org.firstinspires.ftc.robotcore.external.Telemetry;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
 import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
 import org.firstinspires.ftc.vision.VisionPortal;
 import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
 import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
 import org.firstinspires.ftc.vision.tfod.TfodProcessor;
 
 import java.util.List;
 
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
 
 public class RobotHardware {
 
     /* Declare OpMode members. */
     private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
     private final Telemetry telemetry;
     private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
     public AprilTagProcessor aprilTag;
     public TfodProcessor tfod;
     private VisionPortal myVisionPortal;
 
     // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
     // this is only used for Android Studio when using models in Assets.
 //    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
     // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
     // this is used when uploading models directly to the RC using the model upload interface.
     private static final String TFOD_MODEL_FILE = "model_20231220_084721.tflite";
     // Define the labels recognized in the model for TFOD (must be in training order!)
     private static final String[] LABELS = {
             "bb",
             "blue-beacon",
             "rb",
             "red-beacon",
     };
 
     // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
     private DcMotor leftFrontDrive = null;
     private DcMotor leftBackDrive = null;
     private DcMotor rightFrontDrive = null;
     private DcMotor rightBackDrive = null;
     public DcMotor armMotor = null;
     public DcMotor leftIntake = null;
     public DcMotor rightIntake = null;
     public DcMotor oneTimeMotor = null;
     public Servo launchDroneServo = null;
     private Servo leftHand = null;
     private Servo rightHand = null;
     private IMU imu = null;      // Control/Expansion Hub IMU
 
     // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
     public static final double MID_SERVO = 0.5;
     public static final double HAND_SPEED = 0.02;  // sets rate to move servo
     private double headingError = 0;
 
     // These variable are declared here (as class members) so they can be updated in various methods,
     // but still be displayed by sendTelemetry()
     private double targetHeading = 0;
     private double driveSpeed = 0;
     private double turnSpeed = 0;
     // Calculate the COUNTS_PER_INCH for your specific drive train.
     // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     // This is gearing DOWN for less speed and more torque.
     // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
 
     // Rev HD Hex Motor (No Gearbox) : 28 counts/revolution
     // eg: GoBILDA 312 RPM (19.2:1) Yellow Jacket 537.7=28*19.2
     // 40:1 Rev 28*40=1120
     // 20:1 Rev 28*20=560
     static final double COUNTS_PER_MOTOR_REV = 560;
     static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
     static final double WHEEL_DIAMETER_INCHES = 3.78;     // 96cm For figuring circumference
     static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
             (WHEEL_DIAMETER_INCHES * 3.1415);
 
     // These constants define the desired driving/control characteristics
     // They can/should be tweaked to suit the specific robot drive train.
 
     static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
     // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
     // Define the Proportional control coefficient (or GAIN) for "heading control".
     // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
     // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
     // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
     static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
     static final double P_STRAFE_GAIN = 0.025;   // Strafe Speed Control "Gain".
     static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
 
 
     // Define a constructor that allows the OpMode to pass a reference to itself.
     public RobotHardware(LinearOpMode opmode) {
         myOpMode = opmode;
         telemetry = opmode.telemetry;
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
         leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "lfmotor");
         leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "lrmotor");
         rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rfmotor");
         rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rrmotor");
         armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm");
         leftIntake = myOpMode.hardwareMap.get(DcMotor.class, "leftIntake");
         rightIntake = myOpMode.hardwareMap.get(DcMotor.class, "rightIntake");
         oneTimeMotor = myOpMode.hardwareMap.get(DcMotor.class, "otm");
         launchDroneServo = myOpMode.hardwareMap.get(Servo.class, "lds");
 
         oneTimeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         /* The next two lines define Hub orientation.
          * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
          *
          * TODO:  EDIT these two lines to match YOUR mounting configuration.
          */
         //TODO:control hub的朝向记得要改
         RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
         RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
         RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
 
         // Now initialize the IMU with this mounting orientation
         // This sample expects the IMU to be in a REV Hub and named "imu".
         imu = myOpMode.hardwareMap.get(IMU.class, "imu");
         imu.initialize(new IMU.Parameters(orientationOnRobot));
 
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
      * @param lateral Right/Left driving power (-1.0 to 1.0) +ve is forward
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
         telemetry.addData("Speed", "Vy %5.2f, Vx %5.2f, Vr %5.2f ", axial, lateral, yaw);
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
 
 //        leftFrontPower  *=0.5 ;
 //        rightFrontPower *=0.5 ;
 //        leftBackPower   *=0.5 ;
 //        rightBackPower  *=0.5 ;
 
         // TODO: 电机测试代码，反注释下方代码进行测试。
         // 二号手柄按住share键
         // X Y
         // A B
         if(myOpMode.gamepad2.share) {
             leftFrontPower  = myOpMode.gamepad2.x ? 1 : 0;
             rightFrontPower = myOpMode.gamepad2.y ? 1 : 0;
             leftBackPower   = myOpMode.gamepad2.a ? 1 : 0;
             rightBackPower  = myOpMode.gamepad2.b ? 1 : 0;
         }
 
         // Use existing function to drive both wheels.
         setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
     }
 
     /**
      * 单独设置每一个电机的功率。一般情况下无需单独控制单个电机，所以这里是内部函数，外部无法访问。
      *
      * @param leftFrontPower  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
      * @param rightFrontPower Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
      * @param leftBackPower   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
      * @param rightBackPower  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
      */
     private void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
         telemetry.addData("Front Left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
         telemetry.addData("Back  Left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
         // Output the values to the motor drives.
         leftFrontDrive.setPower(leftFrontPower);
         rightFrontDrive.setPower(rightFrontPower);
         leftBackDrive.setPower(leftBackPower);
         rightBackDrive.setPower(rightBackPower);
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
      * isAllBusy
      */
     public boolean isAllBusy() {
         return leftFrontDrive.isBusy() &&
                 leftBackDrive.isBusy() &&
                 rightFrontDrive.isBusy() &&
                 rightBackDrive.isBusy();
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
         rightIntake.setPower(power);
         return this;
     }
 
     public RobotHardware setOneTimeMotorPower(double power) {
         oneTimeMotor.setPower(power);
         return this;
     }
 
     public RobotHardware Launch(double position) {
         launchDroneServo.setPosition(position);
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
 
 
     // ########################################################################################
     // !!!                                    视觉模块                                      !!!!!
     // ########################################################################################
 
 
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
             myVisionPortal = new VisionPortal.Builder()
                     .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                     .addProcessors(tfod, aprilTag)
                     .build();
         } else {
             myVisionPortal = new VisionPortal.Builder()
                     .setCamera(BuiltinCameraDirection.BACK)
                     .addProcessors(tfod, aprilTag)
                     .build();
         }
     }
 
     /**
      * closeVision
      */
     public void closeVision() {
         myVisionPortal.close();
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
 
     public void moveToAprilTag(AprilTagDetection desiredTag, double DESIRED_DISTANCE) {
         // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
         double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
         double yawError = desiredTag.ftcPose.yaw;
         double headingError = desiredTag.ftcPose.bearing;
         driveRobot(
                 rangeError * P_DRIVE_GAIN,
                 yawError * P_STRAFE_GAIN,
                 headingError * P_TURN_GAIN
         );
     }
 
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
         driveRobot(
                 rangeError * P_DRIVE_GAIN,
                 0,
                 headingError * P_TURN_GAIN
         );
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
         telemetry.addData("Yaw/Pitch/Roll",orientation.toString());
         telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
         telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
         telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
         return orientation.getYaw(unit);
     }
 
     /**
      * resetYaw
      */
     public void resetYaw() {
         imu.resetYaw();
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
 
     public RobotHardware A(double d) {
         return driveStraight(0.5, d, getHeading());
     }
     public RobotHardware V(double d) {
         return driveStraight(0.5, -d, getHeading());
     }
     @Deprecated
     public RobotHardware driveStraight(double distance) {
         return driveStraight(0.5, distance, 0);
     }
 
     /**
      * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
      * Move will stop if either of these conditions occur:
      * 1) Move gets to the desired position
      * 2) Driver stops the OpMode running.
      *
      * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
      * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
      * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
      *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
      *                      If a relative angle is required, add/subtract from the current robotHeading.
      */
 
     public RobotHardware driveStraight(double maxDriveSpeed,
                                        double distance,
                                        double heading) {
 
         // Ensure that the OpMode is still active
         if (myOpMode.opModeIsActive()) {
 
             // Determine new target position, and pass to motor controller
             int moveCounts = (int) (distance * COUNTS_PER_INCH);
             setTargetPosition(moveCounts);
 
             setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
 
             // Set the required driving speed  (must be positive for RUN_TO_POSITION)
             // Start driving straight, and then enter the control loop
             maxDriveSpeed = Math.abs(maxDriveSpeed);
             driveRobot(maxDriveSpeed, 0, 0);
 
             // keep looping while we are still active, and BOTH motors are running.
             while (myOpMode.opModeIsActive() && isAllBusy()) {
 
                 // Determine required steering to keep on heading
                 turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
 
                 // if driving in reverse, the motor correction also needs to be reversed
                 if (distance < 0)
                     turnSpeed *= -1.0;
 
                 // Apply the turning correction to the current driving speed.
                 driveRobot(maxDriveSpeed, 0, -turnSpeed);
 //                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
                 telemetry.update();
             }
 
             // Stop all motion & Turn off RUN_TO_POSITION
             stopMotor();
             setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
         }
         return this;
     }
 
 
     public RobotHardware b(double d) {
         return driveStrafe(0.5, d, getHeading());
     }
     public RobotHardware d(double d) {
         return driveStrafe(0.5, -d, getHeading());
     }
     @Deprecated
     public RobotHardware driveStrafe(double distance) {
         return driveStrafe(0.5, distance, 0);
     }
     public RobotHardware driveStrafe(double maxDriveSpeed,
                                      double distance,
                                      double heading
     ) {
 
         // Ensure that the OpMode is still active
         if (myOpMode.opModeIsActive()) {
 
             // Determine new target position, and pass to motor controller
             int moveCounts = (int) (distance * COUNTS_PER_INCH);
             setStrafeTargetPosition(moveCounts);
 
             setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
 
             // Set the required driving speed  (must be positive for RUN_TO_POSITION)
             // Start driving straight, and then enter the control loop
             maxDriveSpeed = Math.abs(maxDriveSpeed);
             driveRobot(0, maxDriveSpeed, 0);
 
             // keep looping while we are still active, and BOTH motors are running.
             while (myOpMode.opModeIsActive() && isAllBusy()) {
 
                 // Determine required steering to keep on heading
                 turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
 
                 // if driving in reverse, the motor correction also needs to be reversed
                 if (distance < 0)
                     turnSpeed *= -1.0;
 
                 // Apply the turning correction to the current driving speed.
                 driveRobot(0, maxDriveSpeed, -turnSpeed);
 //                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
                 telemetry.update();
             }
 
             // Stop all motion & Turn off RUN_TO_POSITION
             stopMotor();
             setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
 
 
     public RobotHardware G(double h) {
         return turnToHeading(0.5,h);
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
      * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
      * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
      *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
      *                     If a relative angle is required, add/subtract from current heading.
      */
     public RobotHardware turnToHeading(double maxTurnSpeed, double heading) {
 
         // Run getSteeringCorrection() once to pre-calculate the current error
         getSteeringCorrection(heading, P_DRIVE_GAIN);
 
         // keep looping while we are still active, and not on heading.
         //绝对值的问题？
         while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
 
             // Determine required steering to keep on heading
             turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
 
             // Clip the speed to the maximum permitted value.
             turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
 
             // Pivot in place by applying the turning correction
             driveRobot(0, 0, -turnSpeed);
 //            telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f",maxTurnSpeed,turnSpeed,heading,getHeading());
             telemetry.update();
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
      * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
      * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
      *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
      *                     If a relative angle is required, add/subtract from current heading.
      * @param holdTime     Length of time (in seconds) to hold the specified heading.
      */
     public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
 
         ElapsedTime holdTimer = new ElapsedTime();
         holdTimer.reset();
 
         // keep looping while we have time remaining.
         while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
             // Determine required steering to keep on heading
             turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
 
             // Clip the speed to the maximum permitted value.
             turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
 
             // Pivot in place by applying the turning correction
             driveRobot(0, 0, turnSpeed);
 
         }
 
         // Stop all motion;
         stopMotor();
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
 
     public RobotHardware sleep(long milliseconds) {
         myOpMode.sleep(milliseconds);
         return this;
     }
 }
 
 