package org.firstinspires.ftc.teamcode.opmode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.test.PutThePixel;
import org.firstinspires.ftc.teamcode.utlity.Alliance;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TeleOp19656 extends LinearOpMode {
    public Alliance teamColor;
    RobotHardware robot = new RobotHardware(this);
    Boolean headless = true;
    Boolean AsianDriverMode = true;
    double speedMul = 1;
    TouchSensor uplimit;
    TouchSensor downlimit;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        runtime.reset();
        // region 程序初始化阶段
        robot.init();
        robot.resetYaw();
        robot.initAprilTagVision();
        robot.setManualExposure(4, 250);
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.STOP);
        uplimit = hardwareMap.get(TouchSensor.class, "uplimit");
        downlimit = hardwareMap.get(TouchSensor.class, "downlimit");
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        // endregion

        // region 运行前准备
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        double armPower = 0;
        double leftIntaker = 0;
        double launchDroneServol = 0;

//        double handOffset   = 0;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        runtime.reset();
        // endregion


        //region 主循环
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //上升沿检测
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //运动

            if (AsianDriverMode) {
                // 日本手
                drive = -sss(gamepad1.left_stick_y);  //Note: pushing stick forward gives negative value
                strafe = sss(gamepad1.left_stick_x);//日本手
                turn = sss(gamepad1.right_stick_x * (1 + 0.5 * (gamepad1.left_stick_y * gamepad1.left_stick_y) + 0.5 * (gamepad1.left_stick_x * gamepad1.left_stick_x))); //日本手 转向补偿（确保旋转角速度一致性）
//            } else {
//                drive = -sss(gamepad1.right_stick_y);//美国手
//                strafe = sss(gamepad1.right_stick_x);//美国手
//                turn = sss(gamepad1.left_stick_x * (1 + 0.5 * (gamepad1.right_stick_y * gamepad1.left_stick_y) + 0.5 * (gamepad1.right_stick_x * gamepad1.left_stick_x)));//美国手
            }
            if (gamepad1.share && gamepad1.options) {
                robot.resetYaw();
            }
            // 基于场地中心坐标系的第三人称操控模式
            if (gamepad1.options && currentGamepad1.x && !previousGamepad1.x) headless = !headless;
            if (gamepad1.options && currentGamepad1.y && !previousGamepad1.y)
                AsianDriverMode = !AsianDriverMode;
            telemetry.addData("Control Mode", headless ? "Headless" : "Normal");
            telemetry.addData("Driver Mode", AsianDriverMode ? "日本手" : "美国手");
            if (headless) {
                robot.driveRobotFieldCentric(drive, strafe, turn);
            } else {
                robot.driveRobot(drive, strafe, turn);
            }
            //微调位置
            if (currentGamepad1.dpad_up) {
                if (teamColor == Alliance.RED) {
                    robot.driveRobot(0, -0.3, 0);
                } else {
                    robot.driveRobot(0, 0.3, 0);
                }
            }
            if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {
                robot.stopMotor();
            }
            if (currentGamepad1.dpad_down) {
                if (teamColor == Alliance.RED) {
                    robot.driveRobot(0, 0.3, 0);
                } else {
                    robot.driveRobot(0, -0.3, 0);
                }
            }
            if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {
                robot.stopMotor();
            }
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                robot.turnToHeading(-90, 0.5, 2);
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                robot.turnToHeading(90, 0.5, 2);
            }

            //进出像素
            leftIntaker = (gamepad2.right_bumper ? -1 : 0) + gamepad2.right_trigger;
//            leftIntaker = (gamepad2.left_bumper ? -1 : 0) + gamepad2.left_trigger;
            robot.setLeftIntakePower(-leftIntaker);
            if (currentGamepad2.right_bumper) {
                robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.OUT);
            } else if (currentGamepad2.left_bumper) {
                robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
                robot.setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT);
                robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.IN);
//            } else if (currentGamepad2.left_trigger > 0.9) {
//                robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
//                robot.setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT);
            } else if (currentGamepad2.left_trigger > 0.1) {
                robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.OUT);
                robot.setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT);
            } else {
                robot.setIntakeRoller(RobotConstants.IntakeRollerPosition.STOP);
                robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.IN);
                robot.setIntakeBackPosition(RobotConstants.IntakeBackPosition.IN);
            }

            //升降摇臂
            if (currentGamepad2.y && !previousGamepad2.y) {
                robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.BACK);
                speedMul = 1;
            } else if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.BOARD);
                speedMul = 0.5;
            } else if (currentGamepad2.a && !previousGamepad2.a) {
                robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.GROUND);
                speedMul = 0.5;
            } else if (currentGamepad2.x && !previousGamepad2.x) {
                robot.setIntakeArmPosition(RobotConstants.IntakeArmPosition.AUTO_BOARD);
                speedMul = 0.5;
            }

            //Intaker 单舵机抬升
            if (gamepad2.b && !gamepad2.share){
                robot.setIntakeFrontPosition(RobotConstants.IntakeFrontPosition.IN);
                robot.setIntakeBackPosition(RobotConstants.IntakeBackPosition.OUT);
            }

            //高挂
            if (gamepad1.right_bumper || (gamepad1.left_bumper && !uplimit.isPressed()||(gamepad2.dpad_down && !uplimit.isPressed()))) {// TODO: 可能有用，暂时保留 (gamepad2.b && !uplimit.isPressed())) {
                armPower = 1;
            } else if (gamepad1.right_trigger > 0.75 || (gamepad1.left_trigger > 0.75 && !downlimit.isPressed())) {
                armPower = -1;
            } else {
                armPower = 0;
            }
            robot.setArmPower(armPower);
            //开门
            if (currentGamepad1.x && !previousGamepad1.x) {
                robot.dooropener.setPosition(0.5);
            } else if (currentGamepad1.y && !previousGamepad1.y) {
                robot.dooropener.setPosition(1);
            } else if (currentGamepad1.a && !previousGamepad1.a){
                robot.dooropener.setPosition(0);
            }



//          TODO:以下代码意义不明，暂时保留
//            if (gamepad1.share && gamepad1.b) {
//                launchDroneServol = 0.45;
//            } else {
//                launchDroneServol = 0.025;
//            }

            //发射无人机
//            launchDroneServol = gamepad2.b && !gamepad2.share? 0.25: 0;  //优雅
            launchDroneServol = gamepad1.b && gamepad1.share? 0.25: 0;
            robot.Launch(launchDroneServol);

            //更新遥测信息
            telemetry.addData("runtime", runtime.milliseconds());
            telemetry.addData("G1 lx ly rx ry", "%.2f %.2f %.2f %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
            telemetry.addData("doorOpenerPosition",robot.dooropener.getPosition());
            telemetry.update();


        }
        // endregion
    }

    private double sss(double v) {
        v = v * speedMul;
        if (v > 0.0) { //若手柄存在中位漂移或抖动就改0.01
            v = 0.87 * v * v * v + 0.13;//0.13是23-24赛季底盘启动需要的功率
        } else if (v < 0.0) { //若手柄存在中位漂移或抖动就改-0.01
            v = 0.87 * v * v * v - 0.13; //三次方是摇杆曲线
        } else {
            // XBOX和罗技手柄死区较大无需设置中位附近
            // 若手柄存在中位漂移或抖动就改成 v*=13
            // 这里的13是上面的0.13/0.01=13
            v = 0;
        }
        return v;
    }

    // region 已废弃
//    public boolean driveToAprilTag(RobotHardware robot, double desiredDistance, int Tag_ID, double Delta_X, double heading) {
//        AprilTagDetection desiredTag = null;
//        double drive = 0;
//        double strafe = 0;
//        double turn = 0;
//        while (!isStopRequested()) {
//            desiredTag = robot.getAprilTag(Tag_ID);
//            if (desiredTag != null && (Math.abs(desiredTag.ftcPose.range - desiredDistance) > 1 || Math.abs(desiredTag.ftcPose.bearing) > 5)) {
//                robot.translateToAprilTag(desiredTag, desiredDistance, Delta_X, heading);
//                gamepad1.rumble(100);
//                return true;
//            } else if (desiredTag != null && Math.abs(desiredTag.ftcPose.range - desiredDistance) < 1 && Math.abs(desiredTag.ftcPose.bearing) < 5) {
//                gamepad1.rumble(100);
//                return true;
//            } else if (desiredTag == null) {
//                drive = -sss(gamepad1.left_stick_y);  //Note: pushing stick forward gives negative value
//                strafe = sss(gamepad1.left_stick_x);//日本手
//                turn = sss(0.8*gamepad1.right_stick_x*(1+0.5*(gamepad1.left_stick_y*gamepad1.left_stick_y)+0.5*(gamepad1.left_stick_x*gamepad1.left_stick_x)));
//                robot.driveRobot(drive,strafe,turn);
//                return false;
//            }
//        }
//        return false;
//    }
    // endregion

    @TeleOp(name = "Tele🔴", group = "A")
    public static class AutoR1 extends TeleOp19656 {
        @Override
        public void runOpMode() {
            teamColor = Alliance.RED;
            super.runOpMode();

        }
    }

    @TeleOp(name = "Tele🔵", group = "A")
    public static class AutoB1 extends TeleOp19656 {
        @Override
        public void runOpMode() {
            teamColor = Alliance.BLUE;
            super.runOpMode();
            ;
        }
    }

}
