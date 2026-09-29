package org.firstinspires.ftc.teamcode.opmode.auto.routes;

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

@Autonomous(name = "R_Far9Ball", preselectTeleOp = "Duo🔴")
public class R_Far9Ball extends LinearOpMode {
    public static String autoStr = "[\n" +
            "    {\n" +
            "        \"x\": 58.25,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": 19.14725148677826,\n" +
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
            "        \"y\": 15.903447769582272,\n" +
            "        \"dy\": 0.5044993683695793,\n" +
            "        \"heading\": -22.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.3,\n" +
            "        \"marker\": \"preloadShoot\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 36.27241896651685,\n" +
            "        \"dx\": 0.4090302586555481,\n" +
            "        \"y\": 22.16770563274622,\n" +
            "        \"dy\": 24.172385215759277,\n" +
            "        \"heading\": 90.01834106445313,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.7,\n" +
            "        \"marker\": \"intake1Start\",\n" +
            "        \"delayAfterArrive\": 0.1\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 36.3540678396821,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": 55.88023602962494,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 90.02090454101563,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.5,\n" +
            "        \"marker\": \"intake1Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 47.72647440433502,\n" +
            "        \"dx\": 20.940119981765747,\n" +
            "        \"y\": 32.57130387425423,\n" +
            "        \"dy\": -41.96856498718262,\n" +
            "        \"heading\": 13.265754699707031,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.7,\n" +
            "        \"marker\": \"beforeShoot2\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 55.00557804107666,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": 15.903447769582272,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": -22.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.7,\n" +
            "        \"marker\": \"shoot2\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 40.7195142460987,\n" +
            "        \"dx\": 5.269463881850243,\n" +
            "        \"y\": 64.71053273975849,\n" +
            "        \"dy\": -0.13472726941108704,\n" +
            "        \"heading\": 18.66899871826172,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"intaker2Start\",\n" +
            "        \"delayAfterArrive\": 0.3\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 61.96859946846962,\n" +
            "        \"dx\": -0.2068568766117096,\n" +
            "        \"y\": 64.8264220058918,\n" +
            "        \"dy\": 0.11499941349029541,\n" +
            "        \"heading\": 18.66899871826172,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 2.0,\n" +
            "        \"marker\": \"intaker2Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 58.38932955265045,\n" +
            "        \"dx\": -4.41362838447094,\n" +
            "        \"y\": 39.60237763822079,\n" +
            "        \"dy\": -29.031143248081207,\n" +
            "        \"heading\": -16.11554718017578,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.6,\n" +
            "        \"marker\": \"beforeShoot3\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 55.00557804107666,\n" +
            "        \"dx\": -0.6110477447509766,\n" +
            "        \"y\": 15.903447769582272,\n" +
            "        \"dy\": 0.047634830698370934,\n" +
            "        \"heading\": -22.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.6,\n" +
            "        \"marker\": \"shoot3\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 35.647982358932495,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": 17.018194183707237,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": -89.14042663574219,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"outline\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    }\n" +
            "]";


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
                                (Globals.RED_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.RED_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 0.985);
                    })

                    .addMarkerTask("start", () -> {
                        robot.subsystem.shooter.setPower(1);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("intake1Start", () -> {
                        // 这里的代码将在到达中间标记点时开启任务
                        robot.command.transportArtifact(1);
                    })

                    .addMarkerTask("intake1Stop", () -> {
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(-1);
                    })

                    .addMarkerTask("beforeShoot2", () -> {
                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.waitFor(200);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("shoot2",()->{
                        double distanceToGoalMM = Math.hypot(
                                (Globals.RED_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.RED_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.subsystem.shooter.staticShoot(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
                        robot.waitFor(500);
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 1);
                    })

                    .addMarkerTask("intaker2Start", () -> {
                        // 这里的代码将在到达中间标记点时开启任务
                        robot.command.transportArtifact(1);
                    })

                    .addMarkerTask("intaker2Stop", () -> {

                    })

                    .addMarkerTask("beforeShoot3", () -> {
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.waitFor(200);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })

                    .addMarkerTask("shoot3",()->{
                        double distanceToGoalMM = Math.hypot(
                                (Globals.RED_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                                (Globals.RED_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
                        );
                        robot.telemetry.addData("CurrDistance", distanceToGoalMM);
                        robot.subsystem.shooter.staticShoot(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM));
                        robot.waitFor(500);
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 1.0);
                    })


                    .execute(autoStr);
            Robot.sleep(30000);  // 让通过dh计时结束自动，以免异步任务没跑完但路径结束被杀掉
        }
        TaskLoopFrame.stopAndClearAll();
    }
}
