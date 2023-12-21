package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name = "OpMode22545", group = "Team22454")
@Disabled
public class OpMode22545 extends LinearOpMode {//这个是给有麦轮的那个机子的，应该是新高三那台

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

    private DcMotor Arm;
    double maxslider = 0F;
    double power_mul = 0.6F;
    double duck_mul = 0.4F;

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
        HardwareManager manager = new HardwareManager(hardwareMap);

        Servo0 = hardwareMap.get(Servo.class, "e0");
        Servo1 = hardwareMap.get(Servo.class, "e1");
        Servo2 = hardwareMap.get(Servo.class, "s2");
        Servo3 = hardwareMap.get(Servo.class, "s3");
        Servo4 = hardwareMap.get(Servo.class, "s4");
        Servo5 = hardwareMap.get(Servo.class, "e5");

        Servo0.setDirection(Servo.Direction.REVERSE);
        Servo1.setDirection(Servo.Direction.REVERSE);

        Arm = hardwareMap.get(DcMotor.class, "arm");
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

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad2.y){
                Servo5.setPosition(0.06);
                Servo1.setPosition(0);


            }else if(gamepad2.b){
                Servo5.setPosition(0.0172);
                Servo1.setPosition(0.435);

            }else if(gamepad2.a){
                Servo5.setPosition(0.073);
                Servo1.setPosition(0.6305);

            }else{
//                Servo5.setPosition(1);
//                Servo1.setPosition(0.3);
//                Arm.setPower(0.5);
//                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                Arm.setTargetPosition(100);
//                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }



            // 爪子舵机
            if (gamepad2.x) {
                Servo0.setPosition(0);
            } else {
                Servo0.setPosition(0.5);
            }
//
            // 手腕
            if (gamepad2.right_bumper) {
                Servo5.setPosition(Servo5.getPosition() - 0.001);
            } else if (gamepad2.dpad_left&& Servo5.getPosition() < 0.5) {
                Servo5.setPosition(Servo5.getPosition() + 0.001);
            }


            // 手肘
            if (gamepad2.dpad_up) {
                Servo1.setPosition(Servo1.getPosition() + 0.001);
            } else if (gamepad2.dpad_down) {
                Servo1.setPosition(Servo1.getPosition() - 0.001);
            }

//             大臂
//            if (gamepad2.left_trigger) {
//                Arm.setPower(gamepad2.left_trigger);
//            } else if (gamepad2.right_bumper) {
//                Arm.setPower(-gamepad2.right_trigger);
//            } else {
//                Arm.setPower(0);
//            }
            Arm.setPower(MathUtils.clamp(gamepad2.left_trigger-gamepad2.right_trigger,-1,1));

//            System.out.printf("======================Servo3 %f%n",Servo3.getPosition());
//
//            // S2 1up0down S0 0up1down S1 0up1down
//            if(gamepad1.dpad_up){
//                Servo0.setPosition(1);
//                sleep(7500);
//                Servo0.setPosition(0.5);
//            }
//            if(gamepad1.dpad_down){
//                Servo0.setPosition(0);
//                sleep(6000);
//                Servo0.setPosition(0.5);
//            }
//            if(gamepad1.dpad_left){
//                Servo1.setPosition(1);
//                sleep(6000);
//                Servo1.setPosition(0.5);
//            }
//            if(gamepad1.dpad_right){
//                Servo1.setPosition(0);
//                sleep(4900);
//                Servo1.setPosition(0.5);
//            }if(gamepad1.left_bumper){
//                Servo2.setPosition(1);
//                sleep(2750);
//                Servo2.setPosition(0.5);
//            }
//            if(gamepad1.right_bumper){
//                Servo2.setPosition(0);
//                sleep(2500);
//                Servo2.setPosition(0.5);
//            }
//            if (gamepad1.dpad_right) power_mul = 1F;
//            if (gamepad1.dpad_left) power_mul = 0.5F;

            //region 读取手柄上的信息
            p1lx = gamepad1.left_stick_x;
            p1ly = -gamepad1.left_stick_y;
            p1rx = gamepad1.right_stick_x;
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
            LFmotor.setPower(power_LFmotor * power_mul);
            RFmotor.setPower(power_RFmotor * power_mul);
            LRmotor.setPower(power_LRmotor * power_mul);
            RRmotor.setPower(power_RRmotor * power_mul);
            //endregion

            // region 输出信息
            telemetry.addData("> 爪子 S0 %f", Servo0.getPosition());
            telemetry.addData("> 手腕 S5 %f", Servo5.getPosition());
            telemetry.addData("> 手肘 S1 %f", Servo1.getPosition());
            telemetry.addData("> 手臂 S1 %f", Arm.getCurrentPosition());
            telemetry.addData("> Motors", "LF (%.2f), RF (%.2f), LR(%.2f), RR(%.2f)", power_LFmotor, power_RFmotor, power_RFmotor, power_RRmotor);
            telemetry.addData("> Mode", power_mul == 1F ? "Fast" : "Slow");
            telemetry.update();
            //endregion
        }
//        Servo5.setPosition(0.8);
//        sleep(1000);
    }
}
