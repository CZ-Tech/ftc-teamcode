package org.firstinspires.ftc.teamcode.opmode.auto.routes;

import static org.firstinspires.ftc.teamcode.common.util.OpModeState.Duo;

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

@Autonomous(name = "R_Far6Ball", preselectTeleOp = "Duo🔴")
public class R_Far6Ball extends LinearOpMode {
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
            "        \"heading\": -21.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.3,\n" +
            "        \"marker\": \"preloadShoot\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 52.16562254726887,\n" +
            "        \"dx\": -7.617035865783691,\n" +
            "        \"y\": 45.93679158389568,\n" +
            "        \"dy\": 65.25721311569214,\n" +
            "        \"heading\": -79.1978759765625,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.6,\n" +
            "        \"marker\": \"\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 40.7195142460987,\n" +
            "        \"dx\": -16.35387086868286,\n" +
            "        \"y\": 65.55933512747288,\n" +
            "        \"dy\": 6.321263998746872,\n" +
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
            "        \"x\": 55.00557804107666,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": 15.903447769582272,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": -21.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.7,\n" +
            "        \"marker\": \"shoot2\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 60.91110306978226,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": 30.863576486706734,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 63.520194351673126,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": 30.863576486706734,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"marker\": \"\",\n" +
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
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 0.989);
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

                        Robot.sleep(200);

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
                        robot.waitFor(800);
                        robot.command.shoot3Times2(Shooter.distanceToRPM(DistanceUnit.MM, distanceToGoalMM) * 1.0);
                    })

                    .addMarkerTask("intaker2Start", () -> {
                        // 这里的代码将在到达中间标记点时开启任务
                        robot.command.transportArtifact(1);
                    })

                    .addMarkerTask("intaker2Stop", () -> {
                        Robot.sleep(200);
                        robot.command.stopTransporting();
                        robot.subsystem.intaker.intakerIn(0);
                        robot.subsystem.shooter.setPower(1);
                        robot.waitFor(200);
                        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
                    })




                    .execute(autoStr);
            Robot.sleep(30000);  // 让通过dh计时结束自动，以免异步任务没跑完但路径结束被杀掉
        }
        TaskLoopFrame.stopAndClearAll();
    }
}
