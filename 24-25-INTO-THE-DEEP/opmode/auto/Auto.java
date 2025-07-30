package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.drive.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;
import org.firstinspires.ftc.teamcode.common.vision.processor.ColorRange;


public class Auto extends LinearOpMode {
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

        robot.subsystem.newclaw.turnTo(0);
        robot.subsystem.newclaw.up();

        robot.pinpointTrajectory.reset();
        waitForStart();

        //路径
        robot.pinpointTrajectory
                .setMode(PinpointTrajectory.Mode.SEQUENTIAL)
                .startMove()

                //挂第一个样本
                .addFunc(() -> robot.subsystem.arm.DunkTop(1))
                .addPoint(1.4, -32.5, 0, 0, 0, 0, robot.command::SlamFromTop)
//                .addPoint(1.4, -32.5, 0, 0, 0, 0)
                .addTime(1.5)
                .addVelocity(50, 25)
                .addPoint(2.45, -25.864, 6.228, 0, 0, 90)

                //推第二个样本
                .addPoint(2.65, -28.35, 13.2, 0, 0, 116.134, () -> robot.waitFor(0).command.left_IngrabberEatForOpMode())
                .addTime(2.9)
                .addPoint(3.5, -14.638, 13, 0, 0, 45)

                //推第三个样本
                .addVelocity(-25, -30)
                .addPoint(4.25, -27.887, 19.4, 0, 0, 125)
                .addTime(4.55)
                .addPoint(5.35, -17.187, 19.546, 0, 0, 45)

                //推第四个样本
                .addVelocity(-50, -20)
                .addPoint(5.95, -38.272, 26.5, 0, 0, 110)
                .addTime(6.25)
                .addPoint(7.05, -3, 26, 0, 0, 30)
                .addTime(7.45)
                .addFunc(() -> robot.waitFor(60).command.left_IngrabberPut())

                //吃第二个样本
                .addPoint(7.6, -15, 30, 0, 0, 180, () -> robot.subsystem.grabber.release())//7.8
                .addTime(8.3)//8.3
                .addPoint(8.6, -0.2, 30, 0, 0, 180, () -> robot.waitFor(270).command.grabup())//8.8
                .addTime(9.13)//9.35

                //挂第二个样本
                .addPoint(9.4, -13.401, 30, 0, 0, 0)//9.8
                .addVelocity(0, -150)
                .addPoint(10.6, -32.25, -2.228, 0, 0, 0, () -> robot.waitFor(675).command.SlamFromTop2())//10.6
                .addTime(12.1)//12.1

                //吃第三个样本
                .addPoint(13.1, -15, 30, 0, 0, 180, () -> robot.subsystem.grabber.release())//12.9
                .addTime(13.8)//13.6
                .addPoint(14.1, -0.3, 30, 0, 0, 180, () -> robot.waitFor(275).command.grabup())//14.1
                .addTime(14.62)//14.65

                //挂第三个样本
                .addPoint(14.8, -13.401, 30, 0, 0, 0)//15.2
                .addVelocity(0, -150)
                .addPoint(16.35, -32.15, -6.228, 0, 0, 0, () -> robot.waitFor(675).command.SlamFromTop2())//15.9
                .addTime(17.85)//17.7

                //吃第四个样本
                .addPoint(18.8, -15, 30, 0, 0, 180, () -> robot.subsystem.grabber.release())
                .addTime(19.6)
                .addPoint(19.9, -0.3, 30, 0, 0, 180, () -> robot.waitFor(275).command.grabup())
                .addTime(20.42)

                //挂第四个样本
                .addPoint(20.55, -13.401, 30, 0, 0, 0)
                .addVelocity(50, -150)
                .addPoint(22.2, -32.15, -9.228, 0, 0, 0, () -> robot.waitFor(685).command.SlamFromTop2())
                .addTime(23.2)

                //吃第五个样本
                .addPoint(24.1, -15, 30, 0, 0, 180, () -> robot.subsystem.grabber.release())
                .addTime(24.9)
                .addPoint(25.2, -0.3, 30, 0, 0, 180, () -> robot.waitFor(275).command.grabup())
                .addTime(25.75)

                //挂第五个样本
                .addPoint(25.9, -13.401, 30, 0, 0, 0)
                .addVelocity(50, -150)
                .addPoint(27.6, -32.25, -12.228, 0, 0, 0, () -> robot.waitFor(690).command.SlamFromTop2())
                .addTime(30)

                .stopMotor()
        ;

        //wait for stop
        while (opModeIsActive());



//        robot.syncRun(()->{
//            robot.waitFor(11.5);
//        });
    }

    @Autonomous(name = "Auto🔴", group = "Auto", preselectTeleOp = "Duo🔴")
    public static class AutoRed extends Auto {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            Globals.targetColor = ColorRange.RED;
            super.runOpMode();
        }
    }
    @Autonomous(name = "Auto🔵", group = "Auto", preselectTeleOp = "Duo🔵")
    public static class AutoBlue extends Auto {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            Globals.targetColor = ColorRange.BLUE;
            super.runOpMode();
        }
    }
}
