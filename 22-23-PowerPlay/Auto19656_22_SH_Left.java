/* Copyright (c) 2019 FIRST. All rights reserved.
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

import android.annotation.SuppressLint;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.HardwareManager;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Auto19656_22_SH_Left", group = "Team19656",preselectTeleOp = "OPMode19656_solo_cai")
@Disabled
public class Auto19656_22_SH_Left extends LinearOpMode {
    private DcMotor LFmotor;
    private DcMotor LRmotor;
    private DcMotor RFmotor;
    private DcMotor RRmotor;

    private Servo Servo0;
    private Servo Servo1;
    private Servo Servo2;
    private Servo Servo3;
    private Servo Servo4;
    private Servo Servo5;
    private Servo Servo6;
    private Servo Servo7;
    private Servo Servo8;
    private Servo Servo9;
    private Servo Servo10;
    private Servo Servo11;
    private Servo Servo12;

    private ColorSensor color, color1,color2;


    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
//    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE = "model_20230224_094826.tflite";


    private static final String[] LABELS = {
            "fz",
            "fz_cone",
            "mc",
            "mc_cone",
            "ys",
            "ys_cone",
            "ys_core"
    };

    private int label = 0;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AdNvurL/////AAABmavZUv6soEt8pmyAvOuZZNQPuYBSs3HMdg/RjpMPcIK/jY6/6swn95uBLVCO5sMtUOjg8E+GHH7atg5YVjz4/TJg5ijDVRScTHNVpxoe+fzqY16UjJAu7DC8S4vVqWx7rAyIq+FsXkB05Wraxmu6DOv+hDAtDcnsbNV9l71VwzoE5IzQlU1rpGsJLXYTgZclWpTCoxTTc8X+1UF4Qjclh5AZj73uODAc/X+w2WNmAPACgVh6g65Ai9s+/X8GcGnTSNm4QdZDA5TMoE3LZjNb4gOyxPqtBB/nPB45iLBU/3IaqueOzuk1ag1/RalLT+N7F/NA84vtm6//rb3IqK7BO47jlroaKbnPkDhGpy3UvUO/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private boolean running_imu =false;
    private BNO055IMU imu;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        HardwareManager manager = new HardwareManager(hardwareMap);

        Servo0 = hardwareMap.get(ServoImplEx.class, "s0");
        Servo1 = hardwareMap.get(ServoImplEx.class, "s1");
        Servo2 = hardwareMap.get(ServoImplEx.class, "s2");
        Servo3 = hardwareMap.get(ServoImplEx.class, "s3");
        Servo4 = hardwareMap.get(ServoImplEx.class, "s4");
        Servo5 = hardwareMap.get(ServoImplEx.class, "s5");
//        Servo6 = hardwareMap.get(ServoImplEx.class, "s6");
//        Servo7 = hardwareMap.get(ServoImplEx.class, "s7");
//        Servo8 = hardwareMap.get(ServoImplEx.class, "s8");
//        Servo9 = hardwareMap.get(ServoImplEx.class, "s9");
//        Servo10 = hardwareMap.get(ServoImplEx.class, "s10");
//        Servo11 = hardwareMap.get(ServoImplEx.class, "s11");
//        Servo12 = hardwareMap.get(ServoImplEx.class, "s12");

        color = hardwareMap.get(ColorSensor.class, "c");
        color1 = hardwareMap.get(ColorSensor.class, "c1");

//        Servo0.setDirection(Servo.Direction.REVERSE);
//        Servo1.setDirection(Servo.Direction.REVERSE);

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

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

//        TODO:修改预载
//        Servo3.setPosition(0.65);
//        Servo4.setPosition(0.35);

        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//        waitForStart();
//        TODO:修改预载
//        Servo3.setPosition(1);
//        Servo4.setPosition(0);

//        while (label == 0 && !isStopRequested()) {
//            if (tfod != null) {
//                // getUpdatedRecognitions() will return null if no new information is available since
//                // the last time that call was made.
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
//
//                    // step through the list of recognitions and display image position/size information for each one
//                    // Note: "Image number" refers to the randomized image orientation/number
//                    double conf = 0;
//                    for (Recognition recognition : updatedRecognitions) {
//                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
//                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
//                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
//                        double height = Math.abs(recognition.getTop() - recognition.getBottom());
//                        if ((recognition.getLabel().equals("fz")||recognition.getLabel().equals("fz_cone")) && recognition.getConfidence() > conf) {
//                            label = 1;
//                            conf = recognition.getConfidence();
//                        }
//                        if ((recognition.getLabel().equals("mc")||recognition.getLabel().equals("mc_cone")) && recognition.getConfidence() > conf) {
//                            label = 2;
//                            conf = recognition.getConfidence();
//                        }
//                        if ((recognition.getLabel().equals("ys")||recognition.getLabel().equals("ys_cone")) && recognition.getConfidence() > conf) {
//                            label = 3;
//                            conf = recognition.getConfidence();
//                        }
//                        telemetry.addData("", " ");
//                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
//                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
//                    }
//                    telemetry.addData("识别的数字", label);
//                    telemetry.update();
//                }
//            }
//        }
//        sleep(500);


        // 向左平移1.5格子，每个格子1100
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setPower(0.3);
        LRmotor.setPower(0.3);
        RFmotor.setPower(0.3);
        RRmotor.setPower(0.3);
        LFmotor.setTargetPosition(1850);
        LRmotor.setTargetPosition(-1850);
        RFmotor.setTargetPosition(-1850);
        RRmotor.setTargetPosition(1850);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 抬升舵机
        Servo2.setPosition(1);
        Servo1.setPosition(1);
        Servo0.setPosition(0);
        sleep(2500);
        Servo2.setPosition(0.5);
        sleep(3250);
        Servo1.setPosition(0.5);
        sleep(750);
        Servo0.setPosition(0.5);

        // 前进到位
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setTargetPosition(1800);
        LRmotor.setTargetPosition(1800);
        RFmotor.setTargetPosition(1800);
        RRmotor.setTargetPosition(1800);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1250);
        LFmotor.setPower(0.2);
        LRmotor.setPower(0.2);
        RFmotor.setPower(0.2);
        RRmotor.setPower(0.2);

        while (color1.alpha() <= 200) sleep(10);
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        LFmotor.setTargetPosition(-600);
        LRmotor.setTargetPosition(600);
        RFmotor.setTargetPosition(600);
        RRmotor.setTargetPosition(-600);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        // 打开爪子
        Servo3.setPosition(0.65);
        Servo4.setPosition(0.45);
        sleep(1000);

        LFmotor.setPower(0.3);
        LRmotor.setPower(0.3);
        RFmotor.setPower(0.3);
        RRmotor.setPower(0.3);

        // 退后一点距离
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setTargetPosition(-300);
        LRmotor.setTargetPosition(-300);
        RFmotor.setTargetPosition(-300);
        RRmotor.setTargetPosition(-300);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        if (label == 3) {

            //平移到3位置
            LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LFmotor.setTargetPosition(-600);
            LRmotor.setTargetPosition(600);
            RFmotor.setTargetPosition(600);
            RRmotor.setTargetPosition(-600);
            LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(3000);
        }
        if (label == 2) {
            //平移到2位置
            LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LFmotor.setTargetPosition(-1650);
            LRmotor.setTargetPosition(1650);
            RFmotor.setTargetPosition(1650);
            RRmotor.setTargetPosition(-1650);
            LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(3000);
        }
        if (label == 1) {
            //平移到1位置
            LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LFmotor.setTargetPosition(-2700);
            LRmotor.setTargetPosition(2700);
            RFmotor.setTargetPosition(2700);
            RRmotor.setTargetPosition(-2700);
            LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(3000);
        }

        Servo2.setPosition(0);
        Servo1.setPosition(0);
        Servo0.setPosition(1);
        sleep(2000);
        Servo2.setPosition(0.5);
        sleep(2250);
        Servo1.setPosition(0.5);
        sleep(750);
        Servo0.setPosition(0.5);

        double power_mul = 0.6F;

        double p1lx;
        double p1rx;
        double p1ly;
        double power_LFmotor;
        double power_LRmotor;
        double power_RFmotor;
        double power_RRmotor;
        double s0=0, s1=0, s2=0 , s3=0;
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "a1");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        while (opModeIsActive()) {
            // 舵机测试
//            if (gamepad2.dpad_down && touch2.getValue() == 0.0) {
//                Servo0.setPosition(1);
//            } else if (gamepad2.dpad_up) {
//                Servo0.setPosition(0);
//            } else {
//                Servo0.setPosition(0.5);
//            }

            if (gamepad2.dpad_up && (color.red() < color.green() * 1.5)) {
                Servo1.setPosition(1);
            } else if (gamepad2.dpad_down && color.red() > 2000) {
                Servo1.setPosition(0);
            } else {
                Servo1.setPosition(0.5);
            }
//            if (gamepad2.dpad_up && touch1.getValue() == 0.0) {
//                Servo2.setPosition(0.8);
//            } else if (gamepad2.dpad_down && analog.getVoltage() < 2.5) {
//                Servo2.setPosition(0.2);
//            } else {
//                Servo2.setPosition(0.5);
//            }

//            if (gamepad2.right_bumper) {
//                Servo3.setPosition(0.9);
//                Servo4.setPosition(0);
//                s3++;
//            } else if (gamepad2.left_bumper) {
//                Servo3.setPosition(0.5);
//                Servo4.setPosition(0.5);
//                s3--;
//            }
            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                Servo3.setPosition(0.5);
                Servo4.setPosition(0.55);
            } else {
                Servo3.setPosition(0.88);
                Servo4.setPosition(0.07);
            }

            if (gamepad1.a) power_mul = 0.6F;
            if (gamepad1.b) power_mul = 0.5F;


            Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("Rotation", "x=%f y=%f z=%f", o.firstAngle, o.secondAngle, o.thirdAngle);

            if (gamepad1.dpad_left && !running_imu) {
                running_imu = true;
                double position_now = o.thirdAngle;
                new Thread(() -> {
                    Orientation o1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    while (Math.abs(position_now - o1.thirdAngle) < 90.0f) {
                        sleep(50);
                        o1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                        LFmotor.setPower(0.5);
                        RFmotor.setPower(-0.5);
                        LRmotor.setPower(0.5);
                        RRmotor.setPower(-0.5);
                    }
                    running_imu = false;
                }).start();
            }
            if (gamepad1.dpad_right && !running_imu) {
                running_imu = true;
                double position_now = o.thirdAngle;
                new Thread(() -> {
                    Orientation o1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    while (Math.abs(position_now - o1.thirdAngle) < 90.0f) {
                        sleep(50);
                        o1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                        LFmotor.setPower(-0.5);
                        RFmotor.setPower(0.5);
                        LRmotor.setPower(-0.5);
                        RRmotor.setPower(0.5);
                    }
                    running_imu = false;
                }).start();
            }

            //region 读取手柄上的信息
            p1lx = gamepad1.left_stick_x;
            p1ly = -gamepad1.left_stick_y;
            if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1)
                p1rx = MathUtils.clamp(-gamepad1.left_trigger + gamepad1.right_trigger, -1, 1);
            else
                p1rx = 0.5 * MathUtils.clamp(-(gamepad1.left_bumper ? 1 : 0) + (gamepad1.right_bumper ? 1 : 0), -1, 1);
            //endregion

            //region 计算得到底盘各电机要输出的功率
            power_LFmotor = Range.clip((p1ly + p1lx + p1rx), -1, 1);
            power_RFmotor = Range.clip((p1ly - p1lx - p1rx), -1, 1);
            power_LRmotor = Range.clip((p1ly - p1lx + p1rx), -1, 1);
            power_RRmotor = Range.clip((p1ly + p1lx - p1rx), -1, 1);

            //region 为各电机输出功率
            LFmotor.setPower(power_LFmotor * power_mul);
            RFmotor.setPower(power_RFmotor * power_mul);
            LRmotor.setPower(power_LRmotor * power_mul);
            RRmotor.setPower(power_RRmotor * power_mul);
            //endregion

//            telemetry.addData("Distance", "%f cm", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Color", "%d %d %d %d", color.red(), color.green(), color.blue(), color.alpha());
            telemetry.addData("Color1", "%d %d %d %d", color1.red(), color1.green(), color1.blue(), color1.alpha());
            // region 输出信息
            telemetry.addData("Servo3 %f", s3);
            telemetry.addData("Servo0 %f", s0);
            telemetry.addData("Servo1 %f", s1);
            telemetry.addData("Servo2 %f", s2);
            telemetry.addData("Motors", "LFmotor (%.2f), RFmotor (%.2f), LRmotor(%.2f), RRmotor(%.2f)", power_LFmotor, power_RFmotor, power_RFmotor, power_RRmotor);
            telemetry.addData("Mode", power_mul == 1F ? "Fast" : "Slow");
            telemetry.update();
            //endregion
            System.out.println("======================Auto19656_22======================");
            System.out.printf("======================LFmotor %d%n", LFmotor.getCurrentPosition());
            System.out.printf("======================LRmotor %d%n", LRmotor.getCurrentPosition());
            System.out.printf("======================RFmotor %d%n", RFmotor.getCurrentPosition());
            System.out.printf("======================RRmotor %d%n", RRmotor.getCurrentPosition());
//            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("LFmotor", LFmotor.getCurrentPosition());
            telemetry.addData("LRmotor", LRmotor.getCurrentPosition());
            telemetry.addData("RFmotor", RFmotor.getCurrentPosition());
            telemetry.addData("RRmotor", RRmotor.getCurrentPosition());
            telemetry.addData("识别的数字", label);
            telemetry.update();
        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
