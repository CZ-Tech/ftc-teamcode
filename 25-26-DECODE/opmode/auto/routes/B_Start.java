package org.firstinspires.ftc.teamcode.opmode.auto.routes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.command.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.command.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.subsystem.Shooter;

@Disabled
@Config
@Autonomous(name = "B_Start")
public class B_Start extends LinearOpMode {
    public static String autoStr = "[\n" +
            "{\n" +
            "        \"x\": 59.0,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -19.14725148677826,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.0,\n" +
            "        \"marker\": \"start\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 55.00557804107666,\n" +
            "        \"dx\": -0.9334439262747765,\n" +
            "        \"y\": -15.903447769582272,\n" +
            "        \"dy\": -0.5044993683695793,\n" +
            "        \"heading\": 22.20111846923828,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.3,\n" +
            "        \"marker\": \"preloadShoot\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    }" +
            "]";

//    public static double ShootRPMAdd1 = -300;
//    public static double ShootRPMAdd2 = -300;
//    public static double ShootRPMAdd3 = -300;


    @Override
    public void runOpMode() {
        // 1. 初始化你的机器人硬件类 (你的 Robot 类需要包含 odo 和 odoDrivetrain)
        Robot robot = new Robot();
        robot.init(this);

        MixedOdo.isPoseInitialized = true;

        // 2. 实例化轨迹控制器
        PinpointTrajectory trajectory = new PinpointTrajectory(robot);

        waitForStart();

        if (opModeIsActive()) {
            // 3. 示例：使用 TrajectoryLoader 实例 API 添加并执行带标记的任务
            new TrajectoryLoader(trajectory)
                    .addMarkerTask("preloadShoot", () -> {
                        double distanceToGoalMM = Math.hypot(
                                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
                    })

                    .addMarkerTask("start", () -> {
                        robot.subsystem.shooter.setPower(1);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("intake1Start", () -> {
                        // 这里的代码将在到达中间标记点时开启任务
                        robot.command.transportArtifact();
                    })

                    .addMarkerTask("intake1Stop", () -> {
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(-1);
                    })

                    .addMarkerTask("beforeShoot1", () -> {
                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.waitFor(200);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("beforeShoot2", () -> {
                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.waitFor(200);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })
                    .addMarkerTask("beforeShoot3", () -> {
                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.waitFor(200);
                        robot.subsystem.belt.down();
                    })

                    .addMarkerTask("Shoot2",()->{
                        double distanceToGoalMM = Math.hypot(
                                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
                    })

                    .addMarkerTask("intake2Start",()->{
                        robot.command.transportArtifact();
                    })

                    .addMarkerTask("intake2Stop", ()->{
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(-1);
                    })

                    .addMarkerTask("shoot3",()->{
                        double distanceToGoalMM = Math.hypot(
                                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
                    })

                    .addMarkerTask("intake3Start",()->{
                        robot.command.transportArtifact();
                        robot.subsystem.intaker.intakerIn(-1);
                    })

                    .addMarkerTask("intake3Stop", ()->{
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(-1);
                    })

                    .addMarkerTask("Shoot4",()->{
                        double distanceToGoalMM = Math.hypot(
                                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
                    })

                    .execute(autoStr);
        }
        TaskLoopFrame.stopAndClearAll();
    }
}


