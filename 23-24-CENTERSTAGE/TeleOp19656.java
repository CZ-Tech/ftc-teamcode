package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "手动阶段", group = "Robot")
//@Disabled
public class TeleOp19656 extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);
    //    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
//    private Recognition desiredTfod = null;     // Used to hold the data for a detected Tfod
//    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        double armPower = 0;
        double leftIntaker = 0;
//        double rightIntaker = 0;
//        double oneTimeMotor = 0;
        double launchDroneServol = 0;
//        double handOffset   = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        robot.initDoubleVision();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
//        int c=robot.armMotor.getCurrentPosition();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // 日本手
            drive = -sss(gamepad1.left_stick_y);  //Note: pushing stick forward gives negative value
            strafe = sss(gamepad1.left_stick_x);//日本手
            turn = sss(0.8*gamepad1.right_stick_x*(1+0.5*(gamepad1.left_stick_y*gamepad1.left_stick_y)+0.5*(gamepad1.left_stick_x*gamepad1.left_stick_x))); //日本手 转向补偿（确保旋转角速度一致性）
//            drive = -sss(gamepad1.right_stick_y);//美国手
//            strafe = sss(gamepad1.right_stick_x);//美国手
//            turn = sss(0.8*gamepad1.left_stick_x*(1+0.5*(gamepad1.right_stick_y*gamepad1.left_stick_y)+0.5*(gamepad1.right_stick_x*gamepad1.left_stick_x)));//美国手

            if (gamepad1.share) {
                robot.resetYaw();
            }


//            desiredTag = robot.getAprilTag(1);
//            desiredTfod = robot.getTfod("Pixel");
//            // 如果按下 Bumper 并且找到了目标，则自动驶向目标。
//            // LB 为 AprilTag，RB 为 Tfod
//            if (gamepad1.left_bumper && desiredTag!=null) {
//                robot.moveToAprilTag(desiredTag,12);
//                telemetry.addData("Control Mode","Auto AprilTag");
//            } else if (gamepad1.right_bumper && desiredTfod!=null) {
//                robot.moveToTfod(desiredTfod,300);
//                telemetry.addData("Control Mode","Auto Tfod");
//            } else {
            // 基于场地中心坐标系的第三人称操控模式
            robot.driveRobotFieldCentric(drive, strafe, turn);
            // 基于机器人中心视角的第一人称操控模式
            //robot.driveRobot(drive, strafe, turn);
//            telemetry.addData("Control Mode", "Manual");
//            } //TODO:加入自动拾取边幕区像素的程序，并设置对应按键

//            List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
//            List<Recognition> currentRecognitions = robot.tfod.getRecognitions();

/*
            // Use gamepad left & right Bumpers to open and close the claw
            // Use the SERVO constants defined in RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range
            if (gamepad1.right_bumper)
                handOffset += robot.HAND_SPEED;
            else if (gamepad1.left_bumper)
                handOffset -= robot.HAND_SPEED;
            handOffset = Range.clip(handOffset, -0.5, 0.5);

            // Move both servos to new position.  Use RobotHardware class
            robot.setHandPositions(handOffset);
*/
            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in RobotHardware class.
//            if (gamepad1.y) {
//                robot.armMotor.setTargetPosition(c + 7200);
//                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.setArmPower(1.0);
//            }
//            if (gamepad1.a){
//                robot.armMotor.setTargetPosition(c);
//                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.setArmPower(1.0);
//            }
//            robot.setArmPower(0.0);

//            if (gamepad1.y) {
//                armPower = 0.45;
//            } else if (gamepad1.a) {
//                armPower = -0.45;
//            } else {
//                armPower = 0;
//            }

            // 抬升
            if (gamepad2.y) {
                armPower = -1;
            } else if (gamepad2.a) {
                armPower = 1;
            } else {
                armPower = 0;
            }
            //开门
            if (gamepad1.x) {
                robot.dooropener.setPosition(1500);
            } else if (gamepad1.y) {
                robot.dooropener.setPosition(2500);
            } else if (gamepad1.a){
                robot.dooropener.setPosition(500);
            }

//            armPower = (gamepad1.y ? -0.45 : 0) + (gamepad1.a ? 0.45 : 0);
            robot.setArmPower(armPower);

            //进出像素
            leftIntaker = gamepad1.left_bumper ? -1 : gamepad1.left_trigger;
            leftIntaker = (gamepad2.left_bumper ? -1 : 0) + gamepad2.left_trigger;
            robot.setLeftIntakePower(-leftIntaker);

//            rightIntaker = gamepad1.right_bumper ? -1 : gamepad1.right_trigger;
//            rightIntaker = (gamepad2.right_bumper ? -1 : 0) + gamepad2.right_trigger;
//            robot.setRightIntakePower(rightIntaker);

            //打斐济
            launchDroneServol = gamepad2.b ? 0.75 : 1; //优雅
            robot.Launch(launchDroneServol);


            //测试模式
//            if(gamepad2.share){
//            if (gamepad2.dpad_up) {
//                oneTimeMotor = 0.25;
//            } else if (gamepad2.dpad_down) {
//                oneTimeMotor = -0.25;
//            } else {
//                oneTimeMotor = 0;
//            }
//
//            robot.setOneTimeMotorPower(oneTimeMotor);
//            }
//            telemetry.addData("Armcount","Armcount = %d", robot.armMotor.getCurrentPosition()-c);

            // Send telemetry messages to explain controls and show robot status


//            telemetry.addData("oneTimeMotor Power", "%.2f", oneTimeMotor);
            telemetry.addData("G1 lx ly rx ry", "%.2f %.2f %.2f %.2f", gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y);

//            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);
            telemetry.update();

        }
//        robot.closeVision();
    }
    private double sss(double v){
        if(v>0.0){ //若手柄存在中位漂移或抖动就改0.01
            v=0.87*v*v*v+0.09;//0.13是23-24赛季底盘启动需要的功率
        }else if (v<0.0) { //若手柄存在中位漂移或抖动就改-0.01
            v=0.87*v*v*v-0.09; //三次方是摇杆曲线
        }else{
            // XBOX和罗技手柄死区较大无需设置中位附近
            // 若手柄存在中位漂移或抖动就改成 v*=13
            // 这里的13是上面的0.13/0.01=13
            v=0;
        }
        return v;
    }
}
