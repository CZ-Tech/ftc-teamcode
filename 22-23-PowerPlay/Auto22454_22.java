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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.HardwareManager;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Auto22454_22", group = "Team22454")
@Disabled
public class Auto22454_22 extends LinearOpMode {
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

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private int label=0;

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

        Servo0 = hardwareMap.get(Servo.class, "e0");
        Servo1 = hardwareMap.get(Servo.class, "e1");
        Servo2 = hardwareMap.get(Servo.class, "e2");
        Servo3 = hardwareMap.get(Servo.class, "e3");
        Servo4 = hardwareMap.get(Servo.class, "e4");
        Servo5 = hardwareMap.get(Servo.class, "e5");

        Servo0.setDirection(Servo.Direction.REVERSE);
        Servo1.setDirection(Servo.Direction.REVERSE);


        LFmotor = manager.getMotor(HardwareManager.Motor.LFmotor);
        LRmotor = manager.getMotor(HardwareManager.Motor.LRmotor);
        RFmotor = manager.getMotor(HardwareManager.Motor.RFmotor);
        RRmotor = manager.getMotor(HardwareManager.Motor.RRmotor);


        LFmotor.setDirection(DcMotor.Direction.REVERSE);
        LRmotor.setDirection(DcMotor.Direction.REVERSE);
        RFmotor.setDirection(DcMotor.Direction.FORWARD);
        RRmotor.setDirection(DcMotor.Direction.FORWARD);

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
            tfod.setZoom(1.0, 16.0/9.0);
        }
        Servo3.setPosition(0.48);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (label==0) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());
                        if (recognition.getLabel().equals("1 Bolt")) label = 1;
                        if (recognition.getLabel().equals("2 Bulb")) label = 2;
                        if (recognition.getLabel().equals("3 Panel")) label = 3;
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


        Servo3.setPosition(0.1);
        LFmotor.setPower(0.3);
        LRmotor.setPower(0.3);
        RFmotor.setPower(0.3);
        RRmotor.setPower(0.3);

        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 前进一小段距离
        LFmotor.setTargetPosition(100);
        LRmotor.setTargetPosition(100);
        RFmotor.setTargetPosition(100);
        RRmotor.setTargetPosition(100);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        // 平移1.5格子，每个格子1100
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setTargetPosition(1650);
        LRmotor.setTargetPosition(-1650);
        RFmotor.setTargetPosition(-1650);
        RRmotor.setTargetPosition(1650);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(5000);
        // 抬升舵机
//        Servo2.setPosition(1);
//        Servo1.setPosition(1);
//        Servo0.setPosition(1);
//        sleep(2500);
//        Servo2.setPosition(0.5);
//        sleep(3250);
//        Servo1.setPosition(0.5);
//        sleep(1750);
//        Servo0.setPosition(0.5);

        // 前进到位
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setTargetPosition(1100);
        LRmotor.setTargetPosition(1100);
        RFmotor.setTargetPosition(1100);
        RRmotor.setTargetPosition(1100);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(3000);

        // 打开爪子
        Servo3.setPosition(0.48);

        // 退后一点距离
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setTargetPosition(-50);
        LRmotor.setTargetPosition(-50);
        RFmotor.setTargetPosition(-50);
        RRmotor.setTargetPosition(-50);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        if(label==3){
            //平移到3位置
            LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LFmotor.setTargetPosition(-550);
            LRmotor.setTargetPosition(550);
            RFmotor.setTargetPosition(550);
            RRmotor.setTargetPosition(-550);
            LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(3000);
        }
        if(label==2){
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
        if(label==1){
            //平移到1位置
            LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LFmotor.setTargetPosition(-2850);
            LRmotor.setTargetPosition(2850);
            RFmotor.setTargetPosition(2850);
            RRmotor.setTargetPosition(-2850);
            LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(3000);
        }


        while (opModeIsActive()) {
            System.out.println("======================Auto19656_22======================");
            System.out.printf("======================LFmotor %d%n",LFmotor.getCurrentPosition());
            System.out.printf("======================LRmotor %d%n",LRmotor.getCurrentPosition());
            System.out.printf("======================RFmotor %d%n",RFmotor.getCurrentPosition());
            System.out.printf("======================RRmotor %d%n",RRmotor.getCurrentPosition());
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
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
