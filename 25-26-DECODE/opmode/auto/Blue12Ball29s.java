package org.firstinspires.ftc.teamcode.opmode.auto;

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

@Autonomous(name = "Blue12Ball29s")
public class Blue12Ball29s extends LinearOpMode {
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
            "    }," +
            "    {\n" +
            "        \"x\": 36.27241896651685,\n" +
            "        \"dx\": 0.4090302586555481,\n" +
            "        \"y\": -22.16770563274622,\n" +
            "        \"dy\": -24.172385215759277,\n" +
            "        \"heading\": -90.01834106445313,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.7,\n" +
            "        \"marker\": \"intake1Start\",\n" +
            "        \"delayAfterArrive\": 0.5\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 36.3540678396821,\n" +
            "        \"dx\": 0.36284446716308594,\n" +
            "        \"y\": -55.88023602962494,\n" +
            "        \"dy\": -1.0449910163879395,\n" +
            "        \"heading\": -90.02090454101563,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.5,\n" +
            "        \"marker\": \"intake1Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 30.99112957715988,\n" +
            "        \"dx\": -15.261065006256104,\n" +
            "        \"y\": -21.262179255485535,\n" +
            "        \"dy\": 10.55859138816595,\n" +
            "        \"heading\": 20.219322204589844,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.8,\n" +
            "        \"marker\": \"beforeShoot1\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -10.784049019217491,\n" +
            "        \"dx\": 1.2772154808044434,\n" +
            "        \"y\": -11.68345770984888,\n" +
            "        \"dy\": 1.175616979598999,\n" +
            "        \"heading\": 41.2796745300293,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.2,\n" +
            "        \"marker\": \"Shoot2\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 11.996327972970903,\n" +
            "        \"dx\": 0.28352298587560654,\n" +
            "        \"y\": -22.465619303286076,\n" +
            "        \"dy\": -28.764731884002686,\n" +
            "        \"heading\": -89.58262634277344,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.5,\n" +
            "        \"marker\": \"intake2Start\",\n" +
            "        \"delayAfterArrive\": 0.5\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 12.430877842009068,\n" +
            "        \"dx\": -0.09433746337890625,\n" +
            "        \"y\": -56.14029943943024,\n" +
            "        \"dy\": -1.0449927300214767,\n" +
            "        \"heading\": -89.56715393066406,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.5,\n" +
            "        \"marker\": \"intake2Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 1.6233906745910645,\n" +
            "        \"dx\": -26.602258801460266,\n" +
            "        \"y\": -21.720924615859985,\n" +
            "        \"dy\": 15.65571403503418,\n" +
            "        \"heading\": 33.12405776977539,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.5,\n" +
            "        \"marker\": \"beforeShoot2\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -13.82655918598175,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -14.628442510962486,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 44.033180236816406,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.5,\n" +
            "        \"marker\": \"shoot3\",\n" +
            "        \"delayAfterArrive\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -11.783330336213112,\n" +
            "        \"dx\": -0.15964984893798828,\n" +
            "        \"y\": -21.224608421325684,\n" +
            "        \"dy\": -12.835066318511963,\n" +
            "        \"heading\": -90.01779174804688,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.5,\n" +
            "        \"marker\": \"intake3Start\",\n" +
            "        \"delayAfterArrive\": 0.5\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -11.505389049649239,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -48.49406370520592,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": -90.0277099609375,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.5,\n" +
            "        \"marker\": \"intake3Stop\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -17.013652205467224,\n" +
            "        \"dx\": -5.734086126089096,\n" +
            "        \"y\": -33.27190697193146,\n" +
            "        \"dy\": 12.160333216190338,\n" +
            "        \"heading\": 35.214115142822266,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.6,\n" +
            "        \"marker\": \"beforeShoot3\",\n" +
            "        \"delayAfterArrive\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -21.407196044921875,\n" +
            "        \"dx\": 0.21924495697021484,\n" +
            "        \"y\": -21.96483564376831,\n" +
            "        \"dy\": -0.5936117470264435,\n" +
            "        \"heading\": 44.59442138671875,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 0.6,\n" +
            "        \"marker\": \"Shoot4\",\n" +
            "        \"delayAfterArrive\": 5.0\n" +
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

