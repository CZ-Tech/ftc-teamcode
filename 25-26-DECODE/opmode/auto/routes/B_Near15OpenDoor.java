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
@Autonomous(name = "B_Near15OpenDoor", preselectTeleOp = "Duo🔵")
public class B_Near15OpenDoor extends LinearOpMode {
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
            "        \"heading\": 47.0,\n" +
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
            "        \"delayAfterArrive\": 0.3\n" +
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
            "        \"x\": -14.191071711480618,\n" +
            "        \"dx\": -0.0,\n" +
            "        \"y\": -14.698556557297707,\n" +
            "        \"dy\": -0.0,\n" +
            "        \"heading\": 47.0,\n" +
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
            "        \"delayAfterArrive\": 0.3\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 12.17964044958353,\n" +
            "        \"dx\": -0.05584716796875,\n" +
            "        \"y\": -60.59730404615402,\n" +
            "        \"dy\": 0.02130280015990138,\n" +
            "        \"heading\": -89.05252075195313,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.2,\n" +
            "        \"marker\": \"intaker2Stop\",\n" +
            "        \"delayAfterArrive\": 0.2\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 6.264018215239048,\n" +
            "        \"dx\": -0.20209646224975586,\n" +
            "        \"y\": -60.80334687232971,\n" +
            "        \"dy\": -0.5389225333929062,\n" +
            "        \"heading\": -34.2861328125,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.3,\n" +
            "        \"marker\": \"pointNearDoor\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -3.962029777467251,\n" +
            "        \"dx\": -9.565868228673935,\n" +
            "        \"y\": -60.78987219184637,\n" +
            "        \"dy\": -0.4715569019317627,\n" +
            "        \"heading\": -0.1628265380859375,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.5,\n" +
            "        \"marker\": \"openDoor2\",\n" +
            "        \"delayAfterArrive\": 0.3\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -15.821311227977276,\n" +
            "        \"dx\": 1.6167664527893066,\n" +
            "        \"y\": -16.28837686777115,\n" +
            "        \"dy\": -0.4715568572282791,\n" +
            "        \"heading\": 47.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"shoot3\",\n" +
            "        \"delayAfterArrive\": 2.7\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 35.99999761581421,\n" +
            "        \"dx\": -0.1197592169046402,\n" +
            "        \"y\": -23.84281349182129,\n" +
            "        \"dy\": -9.5367431640625e-7,\n" +
            "        \"heading\": -89.85572814941406,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"intaker3Start\",\n" +
            "        \"delayAfterArrive\": 0.5\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 36.215568229556084,\n" +
            "        \"dx\": -0.054988861083984375,\n" +
            "        \"y\": -55.342814445495605,\n" +
            "        \"dy\": 0.0185065355617553,\n" +
            "        \"heading\": -89.07307434082031,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"intaker3Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -15.089820146560669,\n" +
            "        \"dx\": -0.11975765228271484,\n" +
            "        \"y\": -15.65119743347168,\n" +
            "        \"dy\": -0.06736526731401682,\n" +
            "        \"heading\": 47.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"shoot4\",\n" +
            "        \"delayAfterArrive\": 2.7\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 43.35446047782898,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -64.23625373840332,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": -28.687896728515625,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.5,\n" +
            "        \"marker\": \"intaker4Start\",\n" +
            "        \"delayAfterArrive\": 0.5\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 62.17832678556442,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -64.06209480762482,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": -28.687896728515625,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"intaker4Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 59.18317836523056,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -21.481083154678345,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 22.20111846923828,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"shoot5\",\n" +
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
                    })

                    .addMarkerTask("openDoor2", () -> {

                        robot.waitFor(400);
                        robot.subsystem.shooter.setPower(1);
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

                    .addMarkerTask("intaker3Start", () -> {
                        intakeOn.set(true);
                        while (intakeOn.get()) robot.command.transportArtifact(1);
                    })

                    .addMarkerTask("intaker3Stop", () -> {
                        intakeOn.set(false);
                        Robot.sleep(200);
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(-1);

                        Robot.sleep(500);

                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("shoot4",()->{
                        double distanceToGoalMM = Math.hypot(
                                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.subsystem.shooter.staticShoot(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
                        robot.waitFor(800);
                        robot.command.shoot3Times_noWait(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 0.97);
                    })

                    .addMarkerTask("intaker4Start", () -> {
                        intakeOn.set(true);
                        while (intakeOn.get()) robot.command.transportArtifact(1);
                    })

                    .addMarkerTask("intaker4Stop", () -> {
                        intakeOn.set(false);
                        Robot.sleep(200);
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(-1);

                        Robot.sleep(500);

                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("shoot5",()->{
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


