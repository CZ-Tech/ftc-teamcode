package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.common.Globals.ODO_COUNTS_PER_INCH;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Robot;

//@Config
@Disabled
@TeleOp(name = "OdoEncoder")
public class OdoEncoderTest extends LinearOpMode {
//    Robot robot = new Robot();

    //Telemetry telemetry;
    //HardwareMap hardwareMap;
//    public DcMotorEx encoderLeft = null;
//    public DcMotorEx encoderRight = null;
//    public DcMotorEx encoderCenter = null;

    public static double DISTANCE = 24;
    public static double SPEED = 0.5;
    Robot robot = new Robot();
    @Override
    public void runOpMode(){
        robot.init(this);
        robot.telemetry.addData("Status", "Stopped");
        robot.telemetry.update();

//        encoderCenter = hardwareMap.get(DcMotorEx.class, "midencoder");
//        encoderLeft = hardwareMap.get(DcMotorEx.class, "lencoder");
//        encoderRight = hardwareMap.get(DcMotorEx.class, "rencoder");
//        robot.odometry.encoderCenter.setDirection(DcMotor.Direction.REVERSE);
//        robot.odometry.encoderLeft.setDirection(DcMotor.Direction.REVERSE);
//        robot.odometry.encoderRight.setDirection(DcMotor.Direction.FORWARD);

//        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odometry.encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odometry.encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odometry.encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odometry.resetPos();
        robot.drivetrain.resetYaw();
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();



        while (opModeIsActive()){
            robot.odometry.update();

            robot.gamepad1.update()
                    .keyPress("y", () -> robot.subsystem.arm.up())
                    .keyUp("y", () -> robot.subsystem.arm.stop())
                    .keyPress("a", () -> robot.subsystem.arm.down())
                    .keyUp("a", () -> robot.subsystem.arm.stop())
//                    .keyDown("b", () -> robot.syncRun(() -> robot.subsystem.grabber.slam(1)))
//                    .keyDown("x", () -> robot.syncRun(() -> robot.subsystem.grabber.vertical()))
            ;

            robot.gamepad2.update()
                    .keyDown("cross", () -> robot.subsystem.grabber.release())
                    .keyDown("square", () -> robot.subsystem.grabber.grab())
//                    .keyDown("circle", () -> robot.subsystem.grabber.half_release())
//                    .keyDown("triangle", () -> robot.subsystem.grabber.upPower())
            ;

            //TODO 测试里程计旋转的编码器位置
            robot.telemetry.addData("Left", robot.odoDrivetrain.getLeftPosition()/ODO_COUNTS_PER_INCH);
            robot.telemetry.addData("Right", robot.odoDrivetrain.getRightPosition()/ODO_COUNTS_PER_INCH);
            robot.telemetry.addData("Center", robot.odoDrivetrain.getCenterPosition()/ODO_COUNTS_PER_INCH);
            robot.telemetry.addData("deltaLeft", robot.odometry.getDeltaLeftPosition()/(2000 / (1.88976378 * Math.PI)));
            robot.telemetry.addData("deltaRight", robot.odometry.getDeltaRightPosition()/(2000 / (1.88976378 * Math.PI)));
            robot.telemetry.addData("deltaCenter", robot.odometry.getDeltaCenterPosition()/(2000 / (1.88976378 * Math.PI)));
            robot.telemetry.addData("IMU", robot.drivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("X Position", robot.odometry.X_Pos);
            robot.telemetry.addData("Y Position", robot.odometry.Y_Pos);
            robot.telemetry.addData("Heading", Math.toDegrees(robot.odometry.heading));
            robot.telemetry.addLine();
            robot.telemetry.addData("Armmotor", robot.subsystem.arm.leftArmMotor.getCurrentPosition());
            robot.telemetry.update();
        }


//        robot.drivetrain.driveStraight(DISTANCE, 0, SPEED);

    }

    private double sss(double v) {
        if (v > 0.0) { //若手柄存在中位漂移或抖动就改0.01
            v = 0.87 * v * v * v + 0.09;//0.13是23-24赛季底盘启动需要的功率
        } else if (v < 0.0) { //若手柄存在中位漂移或抖动就改-0.01
            v = 0.87 * v * v * v - 0.09; //三次方是摇杆曲线
        } else {
            // XBOX和罗技手柄死区较大无需设置中位附近
            // 若手柄存在中位漂移或抖动就改成 v*=13
            // 这里的13是上面的0.13/0.01=13
            v = 0;
        }
        return v;
    }
}
