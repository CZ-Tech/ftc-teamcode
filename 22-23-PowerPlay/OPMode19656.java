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

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.util.HardwareManager;
import org.firstinspires.ftc.teamcode.util.MathHelper;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "OPMode19656", group = "Team19656")
@Disabled
public class OPMode19656 extends LinearOpMode {//这个是给有麦轮的那个机子的，应该是新高三那台

    private DcMotor LFmotor;
    private DcMotor LRmotor;
    private DcMotor RFmotor;
    private DcMotor RRmotor;

    private Servo Servo0;
    private ServoImplEx Servo1;
    private Servo Servo2;
    private Servo Servo3;
    private Servo Servo4;
    private Servo Servo5;

    private ElapsedTime runtime = new ElapsedTime();

    private TouchSensor touch1, touch2;
    private ColorSensor color, color1;
    private DistanceSensor distance;

    double power_mul = 0.6F;

    // region 定义功率
    double power_LFmotor;
    double power_LRmotor;
    double power_RFmotor;
    double power_RRmotor;
    //endregion
    //region 定义手柄上的摇杆上的拨动时的输出的数值
    double p1lx;
    double p1rx;
    double p1ly;

    double s0, s1, s2, s3;

    boolean running_imu = false;
    //endregion

    @Override
    public void runOpMode() {
        HardwareManager manager = new HardwareManager(hardwareMap);

        AnalogInput analog = hardwareMap.get(AnalogInput.class, "a1");

        Servo0 = hardwareMap.get(Servo.class, "s0");
        Servo1 = hardwareMap.get(ServoImplEx.class, "s1");
        Servo2 = hardwareMap.get(Servo.class, "s2");
        Servo3 = hardwareMap.get(Servo.class, "s3");
        Servo4 = hardwareMap.get(Servo.class, "s4");
        Servo5 = hardwareMap.get(Servo.class, "s5");

        BNO055IMU imu = manager.getIMU();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        Servo0.setDirection(Servo.Direction.REVERSE);
        Servo1.setDirection(Servo.Direction.REVERSE);
        Servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));

        touch1 = hardwareMap.get(TouchSensor.class, "t1");
        touch2 = hardwareMap.get(TouchSensor.class, "t2");
        color = hardwareMap.get(ColorSensor.class, "c");
        color1 = hardwareMap.get(ColorSensor.class, "c1");
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
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 舵机测试
            if (gamepad2.dpad_down && touch2.getValue() == 0.0) {
                Servo0.setPosition(1);
            } else if (gamepad2.dpad_up) {
                Servo0.setPosition(0);
            } else {
                Servo0.setPosition(0.5);
            }

            if (gamepad2.dpad_up && (color.red() < color.green() * 1.5)) {
                Servo1.setPosition(1);
            } else if (gamepad2.dpad_down && color.red() > 2000) {
                Servo1.setPosition(0);
            } else {
                Servo1.setPosition(0.5);
            }
            if (gamepad2.dpad_up && touch1.getValue() == 0.0) {
                Servo2.setPosition(0.8);
            } else if (gamepad2.dpad_down && analog.getVoltage() < 2.5) {
                Servo2.setPosition(0.2);
            } else {
                Servo2.setPosition(0.5);
            }

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

            telemetry.addData("Distance", "%f cm", distance.getDistance(DistanceUnit.CM));
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
        }
    }
}
