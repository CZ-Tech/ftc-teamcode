package org.firstinspires.ftc.teamcode.opmode.auto.routes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.command.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.command.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.subsystem.Shooter;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
@Autonomous(name = "B_Near9OpenDoor", preselectTeleOp = "Duo🔵")
public class B_Near9OpenDoor extends LinearOpMode {
    public static String autoStr = "[\n" +
            "    {\n" +
            "        \"x\": -63.5,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -43.05688623338938,\n" +
            "        \"dy\": -0.0,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.0,\n" +
            "        \"marker\": \"start\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -17.39347441494465,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -17.872425138950348,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 45.5,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"shoot1\",\n" +
            "        \"delayAfterArrive\": 2.5\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -11.748502217233181,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -24.35479038953781,\n" +
            "        \"dy\": -0.0,\n" +
            "        \"heading\": -89.39506530761719,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.3,\n" +
            "        \"marker\": \"intaker1Start\",\n" +
            "        \"delayAfterArrive\": 0.5\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -11.964071586728096,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -50.492512226104736,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": -89.697998046875,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"intaker1Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -6.346760131418705,\n" +
            "        \"dx\": 13.944610357284546,\n" +
            "        \"y\": -46.69706004858017,\n" +
            "        \"dy\": 0.6062874346971512,\n" +
            "        \"heading\": -89.36387634277344,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.3,\n" +
            "        \"marker\": \"pointNearDoor\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 1.3881942555308342,\n" +
            "        \"dx\": -0.5389220714569092,\n" +
            "        \"y\": -53.899117052555084,\n" +
            "        \"dy\": -18.121257781982422,\n" +
            "        \"heading\": -89.87832641601563,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.5,\n" +
            "        \"marker\": \"openDoor\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 1.4002455994486809,\n" +
            "        \"dx\": 0.06736531853675842,\n" +
            "        \"y\": -56.599752485752106,\n" +
            "        \"dy\": -9.63323301076889,\n" +
            "        \"heading\": -89.84574890136719,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.5,\n" +
            "        \"marker\": \"openDoor2\",\n" +
            "        \"delayAfterArrive\": 0.3\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -14.191071711480618,\n" +
            "        \"dx\": -0.0,\n" +
            "        \"y\": -14.698556557297707,\n" +
            "        \"dy\": -0.0,\n" +
            "        \"heading\": 45.5,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"shoot2\",\n" +
            "        \"delayAfterArrive\": 2.8\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 12.16843032836914,\n" +
            "        \"dx\": -0.11975765228271484,\n" +
            "        \"y\": -23.970069646835327,\n" +
            "        \"dy\": -0.4715568572282791,\n" +
            "        \"heading\": -90.10919189453125,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.7,\n" +
            "        \"marker\": \"intaker2Start\",\n" +
            "        \"delayAfterArrive\": 0.5\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 12.287424877285957,\n" +
            "        \"dx\": -0.05584716796875,\n" +
            "        \"y\": -56.62275332212448,\n" +
            "        \"dy\": 0.02130280015990138,\n" +
            "        \"heading\": -89.05252075195313,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"intaker2Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -15.821311227977276,\n" +
            "        \"dx\": 1.6167664527893066,\n" +
            "        \"y\": -16.28837686777115,\n" +
            "        \"dy\": -0.4715568572282791,\n" +
            "        \"heading\": 45.5,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"shoot3\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 6.166751869022846,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -17.82013326883316,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 88.6908950805664,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"outline\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    }\n" +
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

        AtomicBoolean intakeOn = new AtomicBoolean(false);

        if (opModeIsActive()) {
            // 3. 示例：使用 TrajectoryLoader 实例 API 添加并执行带标记的任务
            new TrajectoryLoader(trajectory)
                    .addMarkerTask("start", () -> {
                        robot.subsystem.shooter.setPower(1);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("shoot1", () -> {
                        Robot.sleep(300);
                        double distanceToGoalMM = Math.hypot(
                                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.command.shoot3Times_noWait(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 0.998);
                    })

                    .addMarkerTask("intaker1Start", () -> {
                        intakeOn.set(true);
                        while (intakeOn.get()) robot.command.transportArtifact(1);
                    })

                    .addMarkerTask("intaker1Stop", () -> {
                        intakeOn.set(false);
                        Robot.sleep(100);
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(-1);

                        Robot.sleep(500);

                        robot.subsystem.intaker.intakerIn(0);

                    })

                    .addMarkerTask("openDoor2", () -> {

                        robot.waitFor(400);
                        robot.subsystem.shooter.setPower(1);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("shoot2",()->{
                        robot.waitFor(400);
                        double distanceToGoalMM = Math.hypot(
                                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.command.shoot3Times_noWait(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 0.995);
                    })

                    .addMarkerTask("intaker2Start", () -> {
                        intakeOn.set(true);
                        while (intakeOn.get()) robot.command.transportArtifact(1);
                    })

                    .addMarkerTask("intaker2Stop", () -> {
                        intakeOn.set(false);
                        Robot.sleep(100);
                        robot.waitFor(400);
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.waitFor(200);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("shoot3",()->{
                        double distanceToGoalMM = Math.hypot(
                                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.subsystem.shooter.staticShoot(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
                        robot.waitFor(800);
                        robot.command.shoot3Times_noWait(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 0.97);
                    })

                    .execute(autoStr);
        }
        TaskLoopFrame.stopAndClearAll();
    }
}


