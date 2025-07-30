package org.firstinspires.ftc.teamcode.opmode.auto.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;

@Disabled
//@Config
@Autonomous(name = "Auto90", group = "Auto", preselectTeleOp = "Duo")
public class Auto90 extends LinearOpMode {
    Robot robot = new Robot();

    public static double[] startPosition = {0, 0, 0, 0};

    @Override
    public void runOpMode(){
        robot.init(this);
        robot.opModeState = OpModeState.Auto;

        robot.telemetry.addData("Status", "Waiting for start");
        robot.telemetry.update();

//        robot.drivetrain.resetYaw();
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.subsystem.grabber.grab();
        robot.command.IngrabberInit();

        robot.pinpointTrajectory.reset();
        waitForStart();

        robot.pinpointTrajectory.startMove(() -> robot.subsystem.arm.ThrowTop(1))//重置里程计
                .addPoint(1.5, -30,0,0,0,0+90, () -> robot.command.SlamFromTop()) //到达高杆
                .addTime(2.5) //等待挂上第一个样本
                .addVelocity(75, 85) //离开潜水器的初速度
                .addPoint(4.5,-18,22,0,0,155+90,()->robot.command.left_IngrabberEatForOpMode())//走到扫第一个样本的地方
                .addTime(5)
                .addPoint(6.5,-18,22,0,0,30+90, () -> robot.command.left_IngrabberPut())//扫地一个
                .addPoint(7.5,-27,23,0,0,125+90, ()->robot.command.left_IngrabberEatForOpMode())//走到扫第二个的地方
                .addTime(8)
                .addPoint(9.5,-27,25,0,0,30+90, () -> robot.command.left_IngrabberPut())//扫第二个
                .addPoint(11,-19.6,35,0,0,173+90, () -> robot.subsystem.grabber.release())//准备吃第一个样本
                .addPoint(12, 0, 35, 0, 0, 173+90, () -> robot.command.grabup())//吃第一个样本
                .addTime(12.8)
                .addVelocity(-50, -25)
                .addPoint(15.2, -30, -6, 0, 0, 0+90, () -> {
                    robot.waitFor(520);
                    robot.command.SlamFromTop2();
                })//挂第二个样本
//                .stopMotor()
                .addTime(16.2)//等待挂上
                .addPoint(17.7,-19.6,35,0,0,165+90, () -> robot.subsystem.grabber.release())//准备吃第二个样本
//                .addTime(18)
                .addPoint(18.9, 0, 35, 0, 0, 165+90, () -> robot.command.grabup())//吃第二个样本
                .addTime(19.5)
                .addVelocity(-50, -25)
                .addPoint(21.8, -32, -9, 0, 0, 0+90, () -> {
                    robot.waitFor(600);
                    robot.command.SlamFromTop2();
                })//挂第三个样本
//                .stopMotor()
                .addTime(22.8)//等待挂上
                .addPoint(24,-19.6,35,0,0,165+90, () -> robot.subsystem.grabber.release())//准备吃第三个样本
                .addPoint(25.5, 0, 35, 0, 0, 165+90, () -> robot.command.grabup())//吃第三个样本
                .addTime(26.2)
                .addVelocity(-50, -25)
                .addPoint(28.5, -32, -12, 0, 0, 0+90, () -> {
                    robot.waitFor(600);
                    robot.command.SlamFromTop2();
                })//挂第四个样本
                .addTime(30, () -> {
//                    robot.waitFor(250);
//                    robot.command.left_IngrabberEatForOpMode();
                })//等待挂上
//                .addPoint(30, 2, 35, 0, 0, 90)
//                .stopMotor() //停止
        ;

        //wait for stop
        while (opModeIsActive());

//        robot.syncRun(()->{
//            robot.waitFor(11.5);
//        });
    }
}