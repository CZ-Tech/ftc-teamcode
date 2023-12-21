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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")

public class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

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
    //region 定义各变量


    //region 定义时间
    private ElapsedTime runtime = new ElapsedTime();
    //endregion

    //region 定义电机

    private DcMotor LFmotor;
    private DcMotor LRmotor;
    private DcMotor RFmotor;
    private DcMotor RRmotor;
    private DcMotor LLmotor;//left lunch motor
    private DcMotor RLmotor;//right lunch motor
    private DcMotor MLmotor;//middle lunch motor
    private DcMotor MCmotor;//middle collect motor




    //endregion

    // region 定义功率
    double power_LFmotor;
    double power_LRmotor;
    double power_RFmotor;
    double power_RRmotor;
    //endregion
    double lunch_power;
    double power_MLmotor;
    double power_MCmotor;

//endregion
    double label_num=0;
//    public void go(string a){
//
//    }
    public void forward(){
        power_LFmotor = 1;
        power_RFmotor = 1;
        power_LRmotor = 1;
        power_RRmotor = 1;
        LFmotor.setPower(power_LFmotor);
        RFmotor.setPower(power_RFmotor);
        LRmotor.setPower(power_LRmotor);
        RRmotor.setPower(power_RRmotor);
    }
    public void backward(){
        power_LFmotor = 1;
        power_RFmotor = 1;
        power_LRmotor = 1;
        power_RRmotor = 1;
        LFmotor.setPower(power_LFmotor);
        RFmotor.setPower(power_RFmotor);
        LRmotor.setPower(power_LRmotor);
        RRmotor.setPower(power_RRmotor);
    }
    public void left(){
        power_LFmotor = -1;
        power_RFmotor = 1;
        power_LRmotor = 1;
        power_RRmotor = -1;
        LFmotor.setPower(power_LFmotor);
        RFmotor.setPower(power_RFmotor);
        LRmotor.setPower(power_LRmotor);
        RRmotor.setPower(power_RRmotor);
    }
    public void right(){
        power_LFmotor = 1;
        power_RFmotor = -1;
        power_LRmotor = -1;
        power_RRmotor = 1;
        LFmotor.setPower(power_LFmotor);
        RFmotor.setPower(power_RFmotor);
        LRmotor.setPower(power_LRmotor);
        RRmotor.setPower(power_RRmotor);
    }
    public void motorstop(){
        power_LFmotor = 0;
        power_RFmotor = 0;
        power_LRmotor = 0;
        power_RRmotor = 0;
        LFmotor.setPower(power_LFmotor);
        RFmotor.setPower(power_RFmotor);
        LRmotor.setPower(power_LRmotor);
        RRmotor.setPower(power_RRmotor);
    }
    @Override
    public void runOpMode() {
        // region 初始化电机

        //region 配对电机
        //RR:right rear
        //RF:right front
        //LF:left front
        LFmotor = hardwareMap.get(DcMotor.class, "LFmotor");
        RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
        LRmotor = hardwareMap.get(DcMotor.class, "LRmotor");
        RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
        // endregion
//        LLmotor = hardwareMap.get(DcMotor.class, "LLmotor");
//        RLmotor = hardwareMap.get(DcMotor.class, "RLmotor");
//        MLmotor = hardwareMap.get(DcMotor.class, "MLmotor");
//        MCmotor = hardwareMap.get(DcMotor.class, "MCmotor");
        //region 设置电机的转动方向
        LFmotor.setDirection(DcMotor.Direction.FORWARD);
        LRmotor.setDirection(DcMotor.Direction.FORWARD);
        RFmotor.setDirection(DcMotor.Direction.REVERSE);
        RRmotor.setDirection(DcMotor.Direction.REVERSE);
        //endregion
//        LLmotor.setDirection(DcMotor.Direction.FORWARD);
//        RLmotor.setDirection(DcMotor.Direction.REVERSE);
//        MLmotor.setDirection(DcMotor.Direction.REVERSE);
//        MCmotor.setDirection(DcMotor.Direction.REVERSE);

        //region 输出“初始化已完成”
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //endregion
        //endregion


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
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        sleep(5000);
//        while(label_num==0) {?
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if (recognition.getLabel() == "Single") {
                        label_num = 1;

                    } else if (recognition.getLabel() == "Quad") {
                        label_num = 4;
                    }
                }
                telemetry.addData("Status", "label_num %.2f ", label_num);
                telemetry.update();
            }
        }
//    }

        waitForStart();
        if (tfod != null) {
            tfod.shutdown();
        }
        if (label_num==0){
             forward();
             sleep(4000);
             stop();
        }else
        if (label_num==1){
            forward();
            sleep(5000);
            left();
            sleep(1000);
            stop();
        }else
        if (label_num==4){
            forward();
            sleep(6500);
            stop();
        }
        label_num=0;

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

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
