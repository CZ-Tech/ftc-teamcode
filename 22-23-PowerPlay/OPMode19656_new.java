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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.HardwareManager;


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

@TeleOp(name = "OPMode19656_new", group = "Team19656")
@Disabled
public class OPMode19656_new extends LinearOpMode {//这个是给有麦轮的那个机子的，应该是新高三那台

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
            Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("Rotation", "x=%f y=%f z=%f", o.firstAngle, o.secondAngle, o.thirdAngle);
        }
    }
}
