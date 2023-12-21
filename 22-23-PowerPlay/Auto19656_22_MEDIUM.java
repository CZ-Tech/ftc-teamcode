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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
@Autonomous(name = "Auto19656_22_M", group = "Team19656",preselectTeleOp = "OPMode19656_solo_cai")
@Disabled
public class Auto19656_22_MEDIUM extends LinearOpMode {
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

    private TouchSensor touch1, touch2;
    private ColorSensor color, color1;
    private DistanceSensor distance;

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

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        HardwareManager manager = new HardwareManager(hardwareMap);

        Servo0 = hardwareMap.get(Servo.class, "s0");
        Servo1 = hardwareMap.get(Servo.class, "s1");
        Servo2 = hardwareMap.get(Servo.class, "s2");
        Servo3 = hardwareMap.get(Servo.class, "s3");
        Servo4 = hardwareMap.get(Servo.class, "s4");
        Servo5 = hardwareMap.get(Servo.class, "s5");

        touch1 = hardwareMap.get(TouchSensor.class, "t1");
        touch2 = hardwareMap.get(TouchSensor.class, "t2");
        color = hardwareMap.get(ColorSensor.class, "c");
        color1 = hardwareMap.get(ColorSensor.class, "c1");
        distance = hardwareMap.get(DistanceSensor.class, "d1");

        Servo0.setDirection(Servo.Direction.REVERSE);
        Servo1.setDirection(Servo.Direction.REVERSE);

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
        Servo3.setPosition(0.65);
        Servo4.setPosition(0.35);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        Servo3.setPosition(1);
        Servo4.setPosition(0);

        while (label == 0 && !isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    double conf = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());
                        if ((recognition.getLabel().equals("fz")||recognition.getLabel().equals("fz_cone")) && recognition.getConfidence() > conf) {
                            label = 1;
                            conf = recognition.getConfidence();
                        }
                        if ((recognition.getLabel().equals("mc")||recognition.getLabel().equals("mc_cone")) && recognition.getConfidence() > conf) {
                            label = 2;
                            conf = recognition.getConfidence();
                        }
                        if ((recognition.getLabel().equals("ys")||recognition.getLabel().equals("ys_cone")) && recognition.getConfidence() > conf) {
                            label = 3;
                            conf = recognition.getConfidence();
                        }
                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                    }
                    telemetry.addData("识别的数字", label);
                    telemetry.update();
                }
            }
        }
        sleep(500);


        // 平移1.5格子，每个格子1100
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
        sleep(2000);
        Servo2.setPosition(0.5);
        sleep(2250);
        Servo1.setPosition(0.5);
        sleep(750);
        Servo0.setPosition(0.5);

        // 前进到位
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setTargetPosition(1300);
        LRmotor.setTargetPosition(1300);
        RFmotor.setTargetPosition(1300);
        RRmotor.setTargetPosition(1300);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(3000);
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setTargetPosition(-1100);
        LRmotor.setTargetPosition(1100);
        RFmotor.setTargetPosition(1100);
        RRmotor.setTargetPosition(-1100);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setPower(0.2);
        LRmotor.setPower(0.2);
        RFmotor.setPower(0.2);
        RRmotor.setPower(0.2);
        LFmotor.setTargetPosition(550);
        LRmotor.setTargetPosition(550);
        RFmotor.setTargetPosition(550);
        RRmotor.setTargetPosition(550);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        while (distance.getDistance(DistanceUnit.CM) > 16) {
            sleep(10);
            telemetry.addData("LFmotor", LFmotor.getCurrentPosition());
            telemetry.addData("LRmotor", LRmotor.getCurrentPosition());
            telemetry.addData("RFmotor", RFmotor.getCurrentPosition());
            telemetry.addData("RRmotor", RRmotor.getCurrentPosition());
            telemetry.addData("Distance",distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
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
            LFmotor.setTargetPosition(600);
            LRmotor.setTargetPosition(-600);
            RFmotor.setTargetPosition(-600);
            RRmotor.setTargetPosition(600);
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
        if (label == 1) {
            //平移到1位置
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

        Servo2.setPosition(0);
        Servo1.setPosition(0);
        Servo0.setPosition(1);
        sleep(1000);
        Servo2.setPosition(0.5);
        sleep(1250);
        Servo1.setPosition(0.5);
        sleep(750);
        Servo0.setPosition(0.5);

        while (opModeIsActive()) {
            System.out.println("======================Auto19656_22======================");
            System.out.printf("======================LFmotor %d%n", LFmotor.getCurrentPosition());
            System.out.printf("======================LRmotor %d%n", LRmotor.getCurrentPosition());
            System.out.printf("======================RFmotor %d%n", RFmotor.getCurrentPosition());
            System.out.printf("======================RRmotor %d%n", RRmotor.getCurrentPosition());
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
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
