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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.HardwareManager;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test2", group="Linear Opmode")
//@Disabled
public class Test2 extends LinearOpMode {

    private DcMotor LFmotor;
    private DcMotor LRmotor;
    private DcMotor RFmotor;
    private DcMotor RRmotor;
    private DcMotor slidermotor;
    private DcMotor duck;

    double maxslider=0F;
    double power_mul=1F;
    double duck_mul=0.4F;

    // region 定义功率
    double power_LFmotor;
    double power_LRmotor;
    double power_RFmotor;
    double power_RRmotor;
    double power_Rmotor;
    //endregion
    //region 定义手柄上的摇杆上的拨动时的输出的数值
    double p1lx;
    double p1rx;
    double p1ly;
    boolean p1w;
    boolean p1s;
    boolean pla;
    boolean pld;
    double p2ly;
    //endregion

    @Override
    public void runOpMode() {
        HardwareManager manager=new HardwareManager(hardwareMap);

        LFmotor=manager.getMotor(HardwareManager.Motor.LFmotor);
        LRmotor=manager.getMotor(HardwareManager.Motor.LRmotor);
        RFmotor=manager.getMotor(HardwareManager.Motor.RFmotor);
        RRmotor=manager.getMotor(HardwareManager.Motor.RRmotor);
        slidermotor=manager.getMotor(HardwareManager.Motor.Motor0);
        duck=manager.getMotor(HardwareManager.Motor.Motor1);

        LFmotor.setDirection(DcMotor.Direction.REVERSE);
        LRmotor.setDirection(DcMotor.Direction.REVERSE);
        RFmotor.setDirection(DcMotor.Direction.FORWARD);
        RRmotor.setDirection(DcMotor.Direction.FORWARD);
        slidermotor.setDirection(DcMotor.Direction.REVERSE);
        duck.setDirection(DcMotorSimple.Direction.FORWARD);

        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            duck.setPower(gamepad1.right_stick_y*duck_mul);

            if(gamepad1.dpad_up){
                maxslider+=0.1;
                slidermotor.setPower(1);
            }
            else if(gamepad1.dpad_down){
                maxslider-=0.1;
                slidermotor.setPower(-1);
            }
            else
                slidermotor.setPower(0);

            if(gamepad1.a) power_mul=1F;
            if(gamepad1.b) power_mul=0.5F;

            //region 读取手柄上的信息
            p1lx = gamepad1.left_stick_x;
            p1ly = -gamepad1.left_stick_y;
            p1rx = Range.clip(gamepad1.right_stick_x, -1, 1);
//            p1w = gamepad1.dpad_up;
//            p1s = gamepad1.dpad_down;
//            pla = gamepad1.dpad_left;
//            pld = gamepad1.dpad_right;
            //endregion

            //region 计算得到底盘各电机要输出的功率
            power_LFmotor = Range.clip((p1ly + p1lx + p1rx), -1, 1);
            power_RFmotor = Range.clip((p1ly - p1lx - p1rx), -1, 1);
            power_LRmotor = Range.clip((p1ly - p1lx + p1rx), -1, 1);
            power_RRmotor = Range.clip((p1ly + p1lx - p1rx), -1, 1);

//            if (gamepad2.dpad_down){
//                power_Rmotor=-0.8;
//            }else if (gamepad2.dpad_up){
//                power_Rmotor=0.8;
//            }else{
//                power_Rmotor=0;
//            }


//        power_LFmotor = 0.1;
//        power_RFmotor = 0.1;
//        power_LRmotor = 0.1;
//        power_RRmotor = 0.1;
            //region 为各电机输出功率
            LFmotor.setPower(power_LFmotor*power_mul);
            RFmotor.setPower(power_RFmotor*power_mul);
            LRmotor.setPower(power_LRmotor*power_mul);
            RRmotor.setPower(power_RRmotor*power_mul);
            //endregion

            // region 输出信息
            telemetry.addData("Motors", "LFmotor (%.2f), RFmotor (%.2f), LRmotor(%.2f), RRmotor(%.2f)", power_LFmotor, power_RFmotor, power_RFmotor, power_RRmotor);
            telemetry.addData("Distance",maxslider);
            telemetry.addData("Mode",power_mul==1F?"Fast":"Slow");
            telemetry.update();
            //endregion
        }
    }
}
