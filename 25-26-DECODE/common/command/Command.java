package org.firstinspires.ftc.teamcode.common.command;

import static org.firstinspires.ftc.teamcode.common.Globals.THIRD_BALL_COMPENSATION_FACTOR;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.common.util.AutoTask;
import org.firstinspires.ftc.teamcode.common.util.Param;

@AutoTask
@Config
public class Command {

    public static double TURN_RANGE = 3, STEER_RANGE = 1.5;
    private double HEADING_ERROR = 0;
    public static double TURN_SPEED = 0.35, STEER_SPEED = 0.3;

    private final Robot robot;

    public Command(Robot robot) {
        this.robot = robot;
    }

    private void sleep(long milliseconds){
        robot.sleep(milliseconds);
    }


    private double reCulcGamepad(double v) {
        if (v > 0.0) { //若手柄存在中位漂移或抖动就改0.01
            v = 0.87 * v * v * v + 0.09;//0.09是23-24赛季底盘启动需要的功率
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

    private static double normalizeHeading(double rawHeading) {
        // 第一步：对360取模，处理超过360或负数的情况（Java%会保留符号，如-90%360=-90，450%360=90）
        double corrected = rawHeading % 360.0;

        // 第二步：若取模后为负数，加360转为正数（如-90 → 270）
        if (corrected < 0) {
            corrected += 360.0;
        }

        // 最终结果：0 ≤ corrected < 360
        return corrected;
    }

    public Command scrollBack(){
//        robot.subsystem.classifier.in();
        robot.subsystem.door.spin(-1).standBy();
        robot.waitFor(300);
        robot.subsystem.door.stop().end();
        return this;
    }



    public Command pushWheel(){
//        robot.subsystem.classifier.out();
        robot.subsystem.door.spin(1).end();
        robot.waitFor(1000);
        robot.subsystem.door.stop().standBy();

        return this;
    }

    public Command transportArtifact(){
        robot.subsystem.intaker.intakerIn(-1).robot.subsystem.belt.up(0.7);
        return this;
    }

    public Command transportArtifact(float beltPower){
        robot.subsystem.intaker.intakerIn(-1).robot.subsystem.belt.up(beltPower);
        return this;
    }


    public static boolean isSmartTransferring = false;
    public Command smartShootTransfer() {
        robot.telemetry.addData("isShootPrepared", robot.subsystem.shooter.isPrepared());
                                                                                                                                                                           isSmartTransferring = true;
        if (robot.subsystem.shooter.isPrepared()) {
            robot.subsystem.belt.up(1);
            robot.subsystem.door.spin(1);
        }
        else {
            robot.subsystem.belt.stop();
            robot.subsystem.door.spin(0);
        }
        return this;
    }

    public Command stopSmartShootTransfer() {
        robot.telemetry.addData("isShootPrepared", robot.subsystem.shooter.isPrepared());
        isSmartTransferring = false;
        robot.subsystem.belt.stop();
        robot.subsystem.door.spin(0);
        return this;
    }

    public Command leaveArtifact(){
        robot.subsystem.intaker.intakerIn(-1).robot.subsystem.belt.down();
        return this;
    }

    public Command stopTransporting(){
        robot.subsystem.intaker.intakerArmStop().robot.subsystem.belt.stop().robot.subsystem.door.spin(0);
        return this;
    }


    public Command autoSteer(){
//        if (robot.odoDrivetrain.tempAngle == 0){
//            robot.odoDrivetrain.turnTo(robot.teamColor.getBaseAngle());
//            return this;
//        }
//        robot.odoDrivetrain.turnTo(robot.teamColor.getBaseAngle() - robot.odoDrivetrain.tempAngle - robot.odo.getHeading(AngleUnit.DEGREES));
//        robot.limelight.pipelineSwitch(robot.teamColor.getBaseAprilTag() % 20);
        double tx = turnToVision(normalizeHeading(robot.odoDrivetrain.getHeading(AngleUnit.DEGREES)) + robot.gyroTracker.getCurrentOffset(AngleUnit.DEGREES),TURN_RANGE, TURN_SPEED);

//        robot.waitFor(250);

        double[] result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());

//        robot.telemetry.addData("tx", tx);
//        robot.telemetry.update();

//        robot.waitFor(1000);

        robot.odo.update();

//        robot.odoDrivetrain.turnTo(robot.odoDrivetrain.getHeading(AngleUnit.DEGREES) - tx, STEER_RANGE, STEER_SPEED);
//        robot.odoDrivetrain.turnTo(robot.odoDrivetrain.getHeading(AngleUnit.DEGREES) - result[6], STEER_RANGE, STEER_SPEED);

        while(robot.opMode.gamepad1.left_trigger > 0.5 && robot.opMode.opModeIsActive()){
            robot.opMode.gamepad1.rumble(500);
            result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());

            robot.odoDrivetrain.driveRobotFieldCentricWithTeleOpHeadReset(
                    reCulcGamepad(-robot.opMode.gamepad1.left_stick_y),
                    reCulcGamepad(robot.opMode.gamepad1.left_stick_x),
                    Range.clip(result[6] * Globals.STEER_GAIN, -STEER_SPEED, STEER_SPEED)
            );
        }
        robot.opMode.gamepad1.stopRumble();

//        robot.waitFor(1000);
//
//        while (
//                Math.abs(tx) > TURN_RANGE
//                && robot.opMode.opModeIsActive()
//                )
//        {
//            tx = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag())[6];
//
//            robot.odoDrivetrain.driveRobotFieldCentric(
//                    reCulcGamepad(-robot.opMode.gamepad1.left_stick_y),
//                    reCulcGamepad(robot.opMode.gamepad1.left_stick_x),
//                    Range.clip(-tx * Globals.STEER_GAIN, -0.3, 0.3));
//        }
//        robot.odoDrivetrain.stopMotor();
//        robot.limelight.pipelineSwitch(6);
        return this;
    }

    private double turnToVision(double heading, double turn_range, double speed){
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        robot.odo.update();
        HEADING_ERROR = normalizeHeading(robot.odoDrivetrain.getHeading(AngleUnit.DEGREES)) - heading;
        double[] result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());
        while(robot.opMode.opModeIsActive() && runtime.seconds() <= 4 && Math.abs(HEADING_ERROR) > turn_range && result[0] == 0){
            robot.opMode.gamepad1.rumble(500);

            result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());

            robot.odo.update();

//            if (HEADING_ERROR <= 150) HEADING_ERROR = getHeading(AngleUnit.DEGREES) - heading;
            HEADING_ERROR = normalizeHeading(robot.odoDrivetrain.getHeading(AngleUnit.DEGREES)) - heading;

//            robot.telemetry.addData("Unnormalized Heading", robot.odoDrivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("Normalized Heading", robot.odoDrivetrain.getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("Error", HEADING_ERROR);
            robot.telemetry.addData("Time", runtime.seconds());
            robot.telemetry.update();

            robot.odoDrivetrain.driveRobotFieldCentricWithTeleOpHeadReset(
                    reCulcGamepad(-robot.opMode.gamepad1.left_stick_y),
                    reCulcGamepad(robot.opMode.gamepad1.left_stick_x),
                    Range.clip(HEADING_ERROR * Globals.TURN_GAIN, -speed, speed));
        }

//        while (robot.opMode.opModeIsActive() && runtime.seconds() <= 4 && result[0] == 0){
//            result = robot.visionLimelight.getAprilTagResults(robot.teamColor.getBaseAprilTag());
//
//            double turn_speed = robot.gyroTracker.getCurrentOffset(UnnormalizedAngleUnit.DEGREES) < 0 ? speed : -speed;
//
//            robot.odoDrivetrain.driveRobotFieldCentric(
//                    reCulcGamepad(-robot.opMode.gamepad1.left_stick_y),
//                    reCulcGamepad(robot.opMode.gamepad1.left_stick_x),
//                    turn_speed
//            );
//        }

        robot.odoDrivetrain.stopMotor();
        robot.opMode.gamepad1.stopRumble();
        return result[6];
    }


    public Command shoot3Times(double rpm){
        int firstWaitMillis = 0;
        robot.waitFor(firstWaitMillis);//100
        robot.subsystem.shooter.staticShoot(rpm);
        int maxMillisPerWait = 1000;
//        int maxMillisAfterShoot = 2000;
        double t = System.currentTimeMillis();
        while (robot.opMode.opModeIsActive() && System.currentTimeMillis() - t <= maxMillisPerWait * 3 - firstWaitMillis) {
            if (System.currentTimeMillis() - t >= maxMillisPerWait * 2) {
                stopSmartShootTransfer();
                robot.subsystem.door.spin(1);
                robot.subsystem.belt.up();
            } else {
                smartShootTransfer();
                if (System.currentTimeMillis() - t >= maxMillisPerWait * 1.5) {
                    robot.subsystem.shooter.staticShoot(rpm * THIRD_BALL_COMPENSATION_FACTOR);
                }
            }
            robot.waitFor(10);
        }
        robot.waitFor(100);
        stopSmartShootTransfer();
        robot.subsystem.shooter.stop();
        return this;
    }

    public static double waitForUpload = 370;
    public static double shootWaitTime1 = 300;
    public static double shootWaitTime2 = 300;
    public static double shootWaitTime3 = 300;
    public Command shoot3Times2(double v){
        robot.subsystem.shooter.staticShoot(v * 0.995);
        robot.waitFor(shootWaitTime1);//100
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(waitForUpload);
        stopTransporting();
        robot.subsystem.door.spin(0);
//

        robot.subsystem.shooter.staticShoot(v * 0.99);
        robot.waitFor(shootWaitTime2);
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(waitForUpload);
        stopTransporting();
        robot.subsystem.door.spin(0);

        robot.subsystem.shooter.staticShoot(v * 1.045);

        robot.waitFor(shootWaitTime3);
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(waitForUpload*1.8);
        stopTransporting();
        robot.subsystem.door.spin(0);

//        robot.waitFor(3000);
        stopTransporting();

        robot.subsystem.shooter.stop();
        return this;
    }

    public Command shoot3TimesAuto(
            @Param("timeWait0") double timeWait0,
            @Param("factor1")   double factor1,
            @Param("uploadMs1") double uploadMs1,
            @Param("timeWait1") double timeWait1,
            @Param("factor2")   double factor2,
            @Param("uploadMs2") double uploadMs2,
            @Param("timeWait2") double timeWait2,
            @Param("factor3")   double factor3,
            @Param("uploadMs3") double uploadMs3){
        double distance = Math.hypot(
                (Globals.BLUE_GOAL_POS.getX(DistanceUnit.MM) - robot.odo.getPosition().getX(DistanceUnit.MM)),
                (Globals.BLUE_GOAL_POS.getY(DistanceUnit.MM) - robot.odo.getPosition().getY(DistanceUnit.MM))
        );
        double v = Shooter.distanceToRPM(DistanceUnit.MM, distance);
        robot.subsystem.shooter.staticShoot(v * factor1);
        robot.waitFor(timeWait0);//100
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(uploadMs1);
        stopTransporting();
        robot.subsystem.door.spin(0);

        robot.subsystem.shooter.staticShoot(v * factor2);
        robot.waitFor(timeWait1);
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(uploadMs2);
        stopTransporting();
        robot.subsystem.door.spin(0);

        robot.subsystem.shooter.staticShoot(v * factor3);

        robot.waitFor(timeWait2);
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(uploadMs3);
        stopTransporting();
        robot.subsystem.door.spin(0);
        robot.subsystem.shooter.stop();
        return this;
    }


    public Command shoot3Times_noWait(double v){
        robot.subsystem.shooter.staticShoot(v);
        robot.waitFor(300);
        transportArtifact(0.9F);
        robot.subsystem.door.spin(1);
        Robot.sleep(2000);
        stopTransporting();
        robot.subsystem.door.spin(0);
        return this;
    }


    public Command shoot3TimesTele(double v){
        robot.subsystem.shooter.staticShoot(v * 0.995);
        robot.waitFor(shootWaitTime1);//100
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(waitForUpload);
        stopTransporting();
        robot.subsystem.door.spin(0);

        if (!robot.opMode.gamepad1.b) {
            robot.subsystem.shooter.stop();
            return this;
        }
//

        robot.subsystem.shooter.staticShoot(v * 1);
        robot.waitFor(shootWaitTime2);
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(waitForUpload);
        stopTransporting();
        robot.subsystem.door.spin(0);

        if (!robot.opMode.gamepad1.b) {
            robot.subsystem.shooter.stop();
            return this;
        }

        robot.subsystem.shooter.staticShoot(v * 1.03);

        robot.waitFor(shootWaitTime3);
        transportArtifact(1);
        robot.subsystem.door.spin(1);
        robot.waitFor(waitForUpload*1.8);
        stopTransporting();
        robot.subsystem.door.spin(0);

//        robot.waitFor(3000);
        stopTransporting();

        robot.subsystem.shooter.stop();
        return this;
    }

    public Command intakeOn() {
        transportArtifact();
        return this;
    }

    public Command intakeOff() {
        robot.subsystem.door.spin(0);
        robot.command.stopTransporting();
        robot.command.stopSmartShootTransfer();
        return this;
    }

    public Command shoot3Times_TELEOP(double dis){
        robot.odoDrivetrain.stopMotor();
        robot.subsystem.shooter.staticShoot( dis);
        robot.waitFor(1100);
        robot.command.shoot3Times(dis);
        return this;
    }

    public Command classifyArtifact(boolean isDis, double dis){
        int[] pattern = new int[10];
        switch(robot.pattern.getId()){
            case 21:
                pattern[1] = -1;
                pattern[2] = 1;
                pattern[3] = 1;
                break;
            case 22:
                pattern[1] = 1;
                pattern[2] = -1;
                pattern[3] = 1;
                break;
            case 23:
                pattern[1] = 1;
                pattern[2] = 1;
                pattern[3] = -1;
                break;
        }

        int queue = 1;
        boolean leave;

        leave = robot.visionC270.getColor() == pattern[queue];

        robot.subsystem.belt.stop();
        scrollBack();
        robot.waitFor(150);


        if (!leave){
            pushWheel();
            robot.waitFor(250);
            robot.subsystem.belt.up();
            robot.waitFor(300);
            scrollBack();
            robot.subsystem.belt.stop();
        }

        leave = robot.visionC270.getColor() == pattern[queue];

        if (isDis) robot.subsystem.shooter.shoot(dis);
        else robot.subsystem.shooter.staticShoot( dis);
        robot.waitFor(2000);

        while (queue <= 3 && robot.opMode.opModeIsActive()){
            if (leave) {
                robot.subsystem.belt.up();
                robot.waitFor(400);
                leave = robot.visionC270.getColor() == pattern[queue+1] && queue <= 2;
                pushWheel();
                robot.waitFor(250);
                robot.subsystem.belt.stop();
                if (leave) {
                    scrollBack();
                }
                robot.waitFor(300);
                scrollBack();
                robot.waitFor(300);
                robot.subsystem.belt.stop();
                queue++;
            }
            else {

            }

        }

        return this;
    }

    private Command caseGPP(boolean isDis, double dis){
        int[] pattern = {0, -1, 1, 1};
        int queue = 1;
        boolean leave = false;

        if (isDis) robot.subsystem.shooter.shoot(dis);
        else robot.subsystem.shooter.staticShoot( dis);
        robot.waitFor(2000);

        while (queue <= 3 && robot.opMode.opModeIsActive()){
            leave = robot.visionC270.getColor() == pattern[queue];
            if (leave) {

                queue++;
            }
        }

        return this;
    }

    private Command casePGP(boolean isDis, double dis){
        return this;
    }

    private Command casePPG(boolean isDis, double dis){
        return this;
    }

    public Command BeltDown(){
        robot.subsystem.belt.down();
        TaskLoopFrame.runOnce(() -> {
            robot.waitFor(500);
            robot.subsystem.belt.stop();
        });
        return this;
    }

    public Command BeltDown(int millis){
        robot.subsystem.belt.down();
        TaskLoopFrame.runOnce(() -> {
            robot.waitFor(millis);
            robot.subsystem.belt.stop();
        });
        return this;
    }

    public Command beforeShoot(){
        robot.subsystem.intaker.intakerIn(0);
        robot.subsystem.shooter.setPower(1);
//        robot.waitFor(200);
        BeltDown(200);
        return this;
    }

//    public Command stopTrasporting_beforeShoot(){
//        intakeOn.set(false);
//        Robot.sleep(100);
//        robot.waitFor(400);
//        robot.command.stopTransporting();
//        robot.subsystem.intaker.intakerIn(0);
//        robot.subsystem.shooter.setPower(1);
//        robot.waitFor(200);
//        robot.subsystem.belt.down();robot.subsystem.door.spin(-1);
//    }

}
