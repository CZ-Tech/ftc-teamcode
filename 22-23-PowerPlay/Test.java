/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.HardwareManager;

import java.util.Locale;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Test", group = "Team19656")
@Disabled
public class Test extends LinearOpMode {
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private DcMotor LFmotor;
    private DcMotor LRmotor;
    private DcMotor RFmotor;
    private DcMotor RRmotor;
    private ServoImplEx Servo0;
    private ServoImplEx Servo1;
    private ServoImplEx Servo2;
    private ServoImplEx Servo3;
    private ServoImplEx Servo4;
    private ServoImplEx Servo5;
    private DistanceSensor distance;

    @Override
    public void runOpMode() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();


        HardwareManager manager = new HardwareManager(hardwareMap);
        Servo0 = hardwareMap.get(ServoImplEx.class, "s0");//大翻折上
        Servo1 = hardwareMap.get(ServoImplEx.class, "s1");//大翻折左
        Servo2 = hardwareMap.get(ServoImplEx.class, "s2");//大翻折右
        Servo3 = hardwareMap.get(ServoImplEx.class, "s3");//大翻折下
        Servo4 = hardwareMap.get(ServoImplEx.class, "s4");//
        Servo5 = hardwareMap.get(ServoImplEx.class, "s5");
        Servo0.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo3.setPwmRange(new PwmControl.PwmRange(1000, 2500));
        Servo4.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Servo5.setPwmRange(new PwmControl.PwmRange(500, 2500));
        distance = hardwareMap.get(DistanceSensor.class, "d1");


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
        telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
        telemetry.update();

        waitForStart();
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        new Thread(() -> {
            while (!isStopRequested()) {
                telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
                telemetry.update();
                sleep(10);
            }
        }).start();

        //预载
        Servo0.setPosition(0.4);
        Servo1.setPosition(0.5);
        Servo2.setPosition(0.5);
        Servo3.setPosition(0.4);


        // 向左平移1500
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setPower(0.5);
        LRmotor.setPower(0.5);
        RFmotor.setPower(0.5);
        RRmotor.setPower(0.56);
        LFmotor.setTargetPosition(1700);
        LRmotor.setTargetPosition(-1700);
        RFmotor.setTargetPosition(-1700);
        RRmotor.setTargetPosition(1850);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(3000);
        //左右位置调整
        LFmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LFmotor.setPower(0.25);
        RFmotor.setPower(-0.25);
        LRmotor.setPower(-0.25);
        RRmotor.setPower(0.25);
        while (distance.getDistance(DistanceUnit.CM) > 20 && !isStopRequested()) {


            sleep(10);

        }
        LFmotor.setPower(0);
        RFmotor.setPower(0);
        LRmotor.setPower(0);
        RRmotor.setPower(0);
        sleep(100);
        // 前后位置调整
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double d = (distance.getDistance(DistanceUnit.CM) - 9);
        int dd = (int) (20 * d);
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFmotor.setPower(0.25);
        LRmotor.setPower(0.25);
        RFmotor.setPower(0.25);
        RRmotor.setPower(0.25);
        LFmotor.setTargetPosition(dd);
        LRmotor.setTargetPosition(dd);
        RFmotor.setTargetPosition(dd);
        RRmotor.setTargetPosition(dd);
        LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFmotor.setPower(-0.25);
        RFmotor.setPower(-0.25);
        LRmotor.setPower(-0.25);
        RRmotor.setPower(-0.25);

        sleep(1000);


        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        //大翻折拿桶
//        Servo3.setPosition(0.85);
//        Servo0.setPosition(0.4);
//        sleep(1000);
//        Servo3.setPosition(0.9126);
//        sleep(1000);
//        Servo1.setPosition(0);
//        Servo2.setPosition(1);
//        sleep(1000);
//        Servo3.setPosition(0.85);
//        sleep(1000);
//        Servo3.setPosition(0.12);
//        Servo0.setPosition(0.9185);
//        sleep(3000);
//        Servo1.setPosition(0.25);
//        Servo2.setPosition(0.75);
//        sleep(1000);
//
//        Servo3.setPosition(0.85);
//        Servo0.setPosition(0.4);
//        sleep(1000);
//        Servo3.setPosition(0.938);
//        sleep(1000);
//        Servo1.setPosition(0);
//        Servo2.setPosition(1);
//        sleep(1000);
//        Servo3.setPosition(0.85);
//        sleep(1000);
//        Servo3.setPosition(0.12);
//        Servo0.setPosition(0.9185);
//        sleep(3000);
//        Servo1.setPosition(0.25);
//        Servo2.setPosition(0.75);
//        sleep(1000);
//
//        Servo3.setPosition(0.85);
//        Servo0.setPosition(0.4);
//        sleep(1000);
//        Servo3.setPosition(0.96);
//        sleep(1000);
//        Servo1.setPosition(0);
//        Servo2.setPosition(1);
//        sleep(1000);
//        Servo3.setPosition(0.85);
//        sleep(1000);
//        Servo3.setPosition(0.12);
//        Servo0.setPosition(0.9185);
//        sleep(3000);
//        Servo1.setPosition(0.25);
//        Servo2.setPosition(0.75);
//        sleep(1000);
//
//        Servo3.setPosition(0.85);
//        Servo0.setPosition(0.4);
//        sleep(1000);
//        Servo3.setPosition(0.973);
//        sleep(1000);
//        Servo1.setPosition(0);
//        Servo2.setPosition(1);
//        sleep(1000);
//        Servo3.setPosition(0.85);
//        sleep(1000);
//        Servo3.setPosition(0.12);
//        Servo0.setPosition(0.9185);
//        sleep(3000);
//        Servo1.setPosition(0.25);
//        Servo2.setPosition(0.75);
//        sleep(1000);
//
//        Servo3.setPosition(0.85);
//        Servo0.setPosition(0.4);
//        sleep(1000);
//        Servo3.setPosition(1);
//        sleep(1000);
//        Servo1.setPosition(0);
//        Servo2.setPosition(1);
//        sleep(1000);
//        Servo3.setPosition(0.85);
//        sleep(1000);
//        Servo3.setPosition(0.12);
//        Servo0.setPosition(0.9185);
//        sleep(3000);
//        Servo1.setPosition(0.25);
//        Servo2.setPosition(0.75);
//        sleep(1000);

//            telemetry.addData("Distance",d1.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.3f", AngleUnit.DEGREES.normalize(degrees));
    }
}