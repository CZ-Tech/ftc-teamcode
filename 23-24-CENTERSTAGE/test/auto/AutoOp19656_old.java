package org.firstinspires.ftc.teamcode.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Deprecated
@Autonomous(name="自动操控模式_TEST", group="Test", preselectTeleOp = "")
@Disabled
public class AutoOp19656_old extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);
    static final double     DRIVE_SPEED             = 1.0;     // Max driving speed for better distance accuracy.
//    static final double     RUSH_SPEED              = 1.0;     // 冲回后台的速度(乐
    static final double     TURN_SPEED              = 1.0;     // Max Turn speed to limit turn rate

    //    static final String team = "🔴";🔴
//    static final String team = "🔵";🔵
    static final int team = 1; //🔴
    //static final int team = -1; //🔵

    public enum State {
        寻找像素,          //接
        收集像素,          //化
        发射像素,          //发
        //深得马老师真传（乐
    }
    static volatile State state = State.寻找像素;
    static boolean notgohome = true;

    @Override
    public void runOpMode() {

        robot.init();
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        robot
                .resetYaw()
                .turnToHeading(90, 0.3);
//                .holdHeading(0.5,90,1000);

        telemetry.addData("xx %4d",robot.getHeading());
                telemetry.update();
                sleep(5000);
                robot.resetYaw()
                .driveStraight(24, 0, 0.3)
                .sleep(250)
                .driveStrafe(12, 0, 0.3);
//        robot.initDoubleVision();
//        telemetry.addData("Status", "Initialized");
//        // 下面这句话可能没用，可以尝试删除后测试下
//        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        // 下面这句话可能没用，可以尝试删除后测试下
//        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        /*
//        mission 1代表靠近后台的红左
//        mission 2代表靠近后台的红中
//        mission 3代表靠近后台的红右
//        */
//        int MISSION=3;
//        // Wait for the game to start (Display Gyro value while waiting)
//        while (opModeInInit()) {
//            Recognition recognition = robot.getTfod("Pixel");
//            if(recognition!=null){
//                if (recognition.getLeft()<320){
//                    MISSION=1;
//                }else{
//                    MISSION=2;
//                }
//            }
//            telemetry.addData("MISSION", "%4d", MISSION);
//            telemetry.addData(">", "Robot Heading = %4.0f", robot.getHeading());
//            telemetry.update();
//        }

//        int expectAprilTag = MISSION;
//        if (team == 1) {
//            expectAprilTag += 3;
//        }

//        double StrafeMoveDistance = 0.0; //记录机器人平移距离,用于导航回后台


        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)

//        new Thread(() -> {
//            sleep(25000);
//            notgohome = false;
//            //state = State.STATE_3;
//        }).start();
//        runtime.reset();
//        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)
        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review
//        robot.resetYaw();
        //TODO：第一步，上前摆放紫像素
        //TODO：第二步，上板摆放黄像素
        //TODO：第三步，走最靠墙通道回到前台
        //TODO：第四步，启动状态机，开始扫地
        //TODO：第五步，最后5秒走中间回后台停靠
//        robot.driveStrafe(DRIVE_SPEED, 6.0, 0.0, team);
//        robot.driveStraight(DRIVE_SPEED, 24.0, 0.0);
        //sleep(2000);
//        MISSION = 2;
//        switch (MISSION) {
//            case 1:
//                robot.driveStraight(DRIVE_SPEED, 24.0, 0.0);
//                robot.driveStrafe(DRIVE_SPEED, -14.0, 0.0);
//                robot.setLeftIntakePower(-1.0);
//                sleep(500);
//                robot.setLeftIntakePower(0);
//
//                break;
//            case 2:
//                robot.driveStraight(DRIVE_SPEED, 26.0, 0.0);
//                robot.setLeftIntakePower(-1.0);
//                sleep(500);
//                robot.setLeftIntakePower(0);
//                robot.driveStrafe(DRIVE_SPEED, 44.0, 0.0);
//                sleep(500);
//                robot.driveStrafe(DRIVE_SPEED, -1.0, 0.0);
//                robot.driveStraight(DRIVE_SPEED, -24.0, 0.0);
//                robot.driveStrafe(DRIVE_SPEED, 12.0, 0.0);
//                break;
//            case 3:
//                robot.driveStrafe(DRIVE_SPEED, 11.0, 0.0);
//                robot.driveStraight(DRIVE_SPEED, 20.0, 0.0);
//                robot.setLeftIntakePower(-1.0);
//                sleep(500);
//                robot.setLeftIntakePower(0);
//
//                break;
//        }




//        robot.driveStrafe(DRIVE_SPEED, 24.0, 0.0, team);
//        robot.driveStraight(DRIVE_SPEED, 24.0, 0.0);    // Drive Forward 24" (一块地垫2英尺=24英寸）
//        robot.turnToHeading( TURN_SPEED, -45.0);                // Turn  CW to -45 Degrees
//        robot.holdHeading(   TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
//        sleep(2000);
//        robot.driveStraight(DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
//        robot.turnToHeading( TURN_SPEED, 45.0);                // Turn  CCW  to  45 Degrees
//        robot.holdHeading(   TURN_SPEED, 45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
//        sleep(2000);
//        robot.driveStraight(DRIVE_SPEED, 17.0, 45.0);   // Drive Forward 17" at 45 degrees (-12"x and 12"y)
//        robot.turnToHeading( TURN_SPEED, 0.0);                // Turn  CW  to 0 Degrees
//        robot.holdHeading(   TURN_SPEED, 0.0,  1.0);    // Hold  0 Deg heading for 1 second
//        sleep(2000);
//        robot.driveStraight(DRIVE_SPEED,-48.0, 0.0);8    // Drive in Reverse 48" (should return to approx. staring position)

//        while(opModeIsActive() && notgohome) {
//            //break;
//            switch (state) {
//                case 寻找像素:
//                    //寻找像素
//                    //TODO:需要修改
//                    robot.driveStrafe(DRIVE_SPEED, 24.0, 0.0, team);
//                    StrafeMoveDistance += 24.0;
//                    state = State.收集像素;
//                    break;
//                case 收集像素:
//                    //收集像素
//                    state = State.发射像素;
//                    break;
//                case 发射像素:
//                    //发射像素
//                    state = State.寻找像素;
//                    break;
//                default:
//                    break;
//            }
//
//        }
//        if (opModeIsActive()) {
//            if (StrafeMoveDistance <= 48.0) {
//                robot.driveStrafe(RUSH_SPEED, 48.0 - StrafeMoveDistance, 0.0);
//            } else if (StrafeMoveDistance > 48.0) {
//                robot.driveStrafe(RUSH_SPEED, );
//            }
//           !!!!!!!!!!!!!!!!!!!!!!!!!!
//           TODO：在这里加入回后台的程序
//           !!!!!!!!!!!!!!!!!!!!!!!!!!
//        }
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//
//        robot.closeVision();
    }
}
