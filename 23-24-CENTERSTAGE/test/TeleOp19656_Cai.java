package org.firstinspires.ftc.teamcode.test;

import static org.opencv.core.Core.countNonZero;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;
@Deprecated
@TeleOp(name = "手动阶段test_Cai", group = "Robot")
public class TeleOp19656_Cai extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);
    OpenCvWebcam webcam;
    Boolean headless= true;
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
        double rightIntaker = 0;
        double oneTimeMotor = 0;
        double launchDroneServol = 0;
        double shovelposition = 0;
        double czposition = 0;
        int is_ground = 0;
        int is_board = 0;
//        int is_back = 0;
        int is_grab = 0;
        AprilTagDetection desiredTag = null;

//        double handOffset   = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        robot.initAprilTagVision();
        robot.setManualExposure(4, 250);
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.setIntakeRoller(IntakeRollerPosition.STOP);
//        robot.initDoubleVision();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        runtime.reset();

//        int c=robot.armMotor.getCurrentPosition();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // 日本手
//            drive = -sss(gamepad1.left_stick_y);  //Note: pushing stick forward gives negative value
//            strafe = sss(gamepad1.left_stick_x);//日本手
//            turn = sss(0.8*gamepad1.right_stick_x*(1+0.5*(gamepad1.left_stick_y*gamepad1.left_stick_y)+0.5*(gamepad1.left_stick_x*gamepad1.left_stick_x))); //日本手 转向补偿（确保旋转角速度一致性）
            drive = -sss(gamepad1.right_stick_y);//美国手
            strafe = sss(gamepad1.right_stick_x);//美国手
            turn = sss(0.8*gamepad1.left_stick_x*(1+0.5*(gamepad1.right_stick_y*gamepad1.left_stick_y)+0.5*(gamepad1.right_stick_x*gamepad1.left_stick_x)));//美国手
            robot.updateButtonState();

            if (gamepad1.share&&gamepad1.options) {
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
            if(gamepad1.share&& gamepad1.x)headless=true;
            if(gamepad1.share&& gamepad1.y)headless=false;
            telemetry.addData("Control Mode", headless?"Headless":"Normal");
            if(headless) {
                robot.driveRobotFieldCentric(drive, strafe, turn);
            }else{
                robot.driveRobot(drive, strafe, turn);
            }
//            if (gamepad1.dpad_up) {
//                driveToAprilTag(robot, 10, 2,0, -90);
//                robot.turnToHeading(0.3,-90);
//                PutThePixel.putThePixel(robot,IntakeArmPosition.BOARD);
//            }
//            if (gamepad1.dpad_left) {
//                driveToAprilTag(robot, 12, 1,-2, -90);
//                robot.turnToHeading(0.3,-90);
//                PutThePixel.putThePixel(robot,IntakeArmPosition.BOARD);
//            }
//            if (gamepad1.dpad_right) {
//                driveToAprilTag(robot, 12, 3,2, -90);
//                robot.turnToHeading(0.3,-90);
//                PutThePixel.putThePixel(robot,IntakeArmPosition.BOARD);
//            }
            if (currentGamepad1.right_bumper) {
                robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
                robot.setIntakeBackPosition(IntakeBackPosition.OUT);
                robot.setIntakeRoller(IntakeRollerPosition.IN);
            }
            else if (currentGamepad1.right_trigger>0.5){
                robot.setIntakeFrontPosition(IntakeFrontPosition.OUT);
                robot.setIntakeBackPosition(IntakeBackPosition.OUT);
            }
            else {robot.setIntakeRoller(IntakeRollerPosition.STOP);
            robot.setIntakeFrontPosition(IntakeFrontPosition.IN);
            robot.setIntakeBackPosition(IntakeBackPosition.IN);
            }
            if (currentGamepad1.a && !previousGamepad1.a) {
                robot.setIntakeArmPosition(IntakeArmPosition.BACK);
            }
            else if (currentGamepad1.y && !previousGamepad1.y) {
                robot.setIntakeArmPosition(IntakeArmPosition.BOARD);
            }
            else if (currentGamepad1.x && !previousGamepad1.x) {
                robot.setIntakeArmPosition(IntakeArmPosition.GROUND);
            }
            else if (currentGamepad1.b && !previousGamepad1.b) {
                robot.setIntakeArmPosition(IntakeArmPosition.AUTO_BOARD);
            }
            // 基于机器人中心视角的第一人称操控模式
            //robot.driveRobot(drive, strafe, turn);

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
//
//            if (gamepad1.y) {
//                armPower = 0.45;
//            } else if (gamepad1.a) {
//                armPower = -0.45;
//            } else {
//                armPower = 0;
//            }

            // TODO:抬升
//            if (gamepad2.y) {
//                armPower = 0.5;
//            } else if (gamepad2.a) {
//                armPower = -0.5;
//            } else {
//                armPower = 0;
//            }

//            armPower = (gamepad1.y ? -0.45 : 0) + (gamepad1.a ? 0.45 : 0);
//            robot.setArmPower(armPower);

            //进出像素
            leftIntaker = (gamepad1.left_bumper ? -1 : 0) + gamepad1.left_trigger;
//            leftIntaker = (gamepad2.left_bumper ? -1 : 0) + gamepad2.left_trigger;
            robot.setLeftIntakePower(-leftIntaker);

//            rightIntaker = gamepad1.right_bumper ? -1 : gamepad1.right_trigger;
//            rightIntaker = (gamepad2.right_bumper ? -1 : 0) + gamepad2.right_trigger;
//            robot.setRightIntakePower(rightIntaker);

            //发射无人机
//            if (gamepad2.b) {
//                launchDroneServol = 0.75;
//            }
//            else {
//                launchDroneServol = 1;
//            }
//            launchDroneServol = gamepad2.b ? 0.7 : 0.95;  //优雅
//            robot.Launch(launchDroneServol);


            //测试模式
//            if(gamepad2.share){
//            if (gamepad2.dpad_up) {
//                oneTimeMotor = 0.25;
//            } else if (gamepad2.dpad_down) {
//                oneTimeMotor = -0.25;
//            } else {
//                oneTimeMotor = 0;
//            }
//            robot.setOneTimeMotorPower(oneTimeMotor);

//            if(gamepad2.dpad_up){
//                robot.setServoPosition(robot.yb,0.350);
//            }else{
//                robot.setServoPosition(robot.yb,0.9);
//            }

//            if(robot.dpad_down_once()) {
//                ++is_ground;
//            }
//            if(robot.dpad_up_once()) {
//                ++is_board;
//            }
//            if (robot.y_once()) {
//                ++is_grab;
//            }
//
//            if (is_ground % 2 == 1) {
//                robot.yb.setPosition(0.0625);
//            } else if (is_board % 2 == 1) {
//                robot.yb.setPosition(0.4085);
//            } else {
//                robot.yb.setPosition(0.979);
//            }
//
//            if (is_grab % 2 == 1) {
//                robot.cz.setPosition(0.1315);
//            } else {
//                robot.cz.setPosition(0.0635);
//            }


//            }
//            telemetry.addData("Armcount","Armcount = %d", robot.armMotor.getCurrentPosition()-c);

            // Send telemetry messages to explain controls and show robot status


            telemetry.addData("oneTimeMotor Power", "%.2f", oneTimeMotor);
            telemetry.addData("G1 lx ly rx ry", "%.2f %.2f %.2f %.2f", gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y);
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());

//            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);
            telemetry.update();

        }

//        robot.closeVision();
        class SamplePipeline extends OpenCvPipeline
        {
            @Override
            public Mat processFrame(Mat input)
            {
                countNonZero(input);
                Imgproc.rectangle(
                        input,
                        new Point(
                                input.cols()/4,
                                input.rows()/4),
                        new Point(
                                input.cols()*(3f/4f),
                                input.rows()*(3f/4f)),
                        new Scalar(0, 255, 0), 4);

                return input;
            }
        }
    }
    private double sss(double v){
        if(v>0.0){ //若手柄存在中位漂移或抖动就改0.01
            v=0.5*v*v*v+0.13;//0.13是23-24赛季底盘启动需要的功率
        }else if (v<0.0) { //若手柄存在中位漂移或抖动就改-0.01
            v=0.5*v*v*v-0.13; //三次方是摇杆曲线
        }else{
            // XBOX和罗技手柄死区较大无需设置中位附近
            // 若手柄存在中位漂移或抖动就改成 v*=13
            // 这里的13是上面的0.13/0.01=13
            v=0;
        }
        return v;
    }
    public void driveToAprilTag(RobotHardware robot, double desiredDistance, int Tag_ID, double Delta_X, double heading) {
        AprilTagDetection desiredTag = null;
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        while (!isStopRequested()) {
            desiredTag = robot.getAprilTag(Tag_ID);
            if (desiredTag != null
                    && (Math.abs(desiredTag.ftcPose.range - desiredDistance) > 1
                    || Math.abs(desiredTag.ftcPose.bearing) > 5)) {
                robot.translateToAprilTag(desiredTag, desiredDistance, Delta_X, heading);
            }
            else if (desiredTag != null && Math.abs(desiredTag.ftcPose.range - desiredDistance) < 1
                    && Math.abs(desiredTag.ftcPose.bearing) < 5)
            {
                break;
            }
            else if (desiredTag == null){
                drive = -sss(gamepad1.left_stick_y);  //Note: pushing stick forward gives negative value
                strafe = sss(gamepad1.left_stick_x);//日本手
                turn = sss(0.8*gamepad1.right_stick_x*(1+0.5*(gamepad1.left_stick_y*gamepad1.left_stick_y)+0.5*(gamepad1.left_stick_x*gamepad1.left_stick_x)));
                robot.driveRobot(drive,strafe,turn);
            }
        }
    }
}
