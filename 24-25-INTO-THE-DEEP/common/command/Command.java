package org.firstinspires.ftc.teamcode.common.command;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.subsystem.Claw;

//@Confg
public class Command {

    public double WaitForPutTime = 1500.0;
    private final Robot robot;

    public static long VVV=450;
    public Command(Robot robot) {
        this.robot = robot;
    }

   //stretcher向前吃sample
    public Command right_IngrabberEatForOpMode(){
        robot.syncRun(() -> {
            robot.waitFor(200);
            robot.subsystem.stretcher.RightExpandForward();
        },
                () -> robot.subsystem.claw.release(),
                () ->{
//            robot.subsystem.claw.release();
            robot.subsystem.claw.paralle();
//            robot.waitFor(750);
//            robot.subsystem.claw.up();
        });
        return this;
    }

    //strecher向后放sample
    public Command right_IngrabberPut(){
        robot.syncRun(()->{
            robot.waitFor(200);
            robot.subsystem.stretcher.RightExpandBack();
            },
                () -> robot.subsystem.claw.grab(),
        ()->{

            robot.subsystem.claw.paralle();
            robot.waitFor(750);
            robot.subsystem.claw.up();
        });
        return this;
    }

    public Command updateTelemetry(){
        robot.telemetry.addData("Stat", robot.limelight.getStat() ? "Found" : "Not Found");
        robot.telemetry.addData("X", robot.limelight.getCx());
        robot.telemetry.addData("Y", robot.limelight.getCy());
        robot.telemetry.addData("Tx", robot.limelight.getTx());
        robot.telemetry.addData("Ty", robot.limelight.getTy());
        robot.telemetry.addData("Cx", robot.limelight.getCx() - (int)robot.limelight.getWidth() / 2);
        robot.telemetry.addData("Cy", robot.limelight.getCy() - (int)robot.limelight.getHeight() / 2);
        robot.telemetry.addData("Angle", robot.limelight.getAngle());
        robot.telemetry.addData("Claw Position", Claw.calcTurningPos(robot.limelight.getAngle()));
        robot.telemetry.update();
        return this;
    }

    public static double P_MOVE_STEER_GAIN = 0.05;

    public static double Y_START_POWER = 0.15;
    public static double X_START_POWER = 0.25;

    public static double Y_MAX_POWER = 0.19;
    public static double X_MAX_POWER = 0.29;

    public static double motorVoltage = 12.26;
    public Command rightClawRunForSamples(){
        robot.syncRun(
                () -> robot.subsystem.stretcher.RightExpandForward()
//                () -> robot.subsystem.claw.release(),
//                () -> {
//                    while(robot.opMode.opModeIsActive()) robot.limelight.update();
//                }
        );

        robot.limelight.start();

        robot.subsystem.claw.release();
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        boolean found = false;
        double angle = 0;

        robot.waitFor(100);

        while(!robot.limelight.getStat() && robot.opMode.opModeIsActive() && runtime.seconds() <= 1.0){
            robot.limelight.update();
            updateTelemetry();
            if (robot.limelight.getStat()) {
                found = true;
                angle = robot.limelight.getAngle();
            }
        }

        if (!found){
            right_IngrabberPut();
            return this;
        }

        robot.subsystem.stretcher.rightStop();

        runtime.reset();
        while(runtime.seconds() <= 0.5){
            robot.limelight.update();
            updateTelemetry();
        }

        if (!robot.limelight.getStat()){
            right_IngrabberPut();
            return this;
        }

        runtime.reset();

        while((robot.limelight.getTy() <= 14.5 || robot.limelight.getTy() >= 15.5) && robot.opMode.opModeIsActive() && robot.limelight.getStat() && runtime.seconds() <= 7){
            double speed = (robot.limelight.getTy() - 15) * P_MOVE_STEER_GAIN;
            if (speed > -Y_START_POWER && speed < 0) speed = -Y_START_POWER;
            else if (speed < Y_START_POWER && speed > 0) speed = Y_START_POWER;
            else if (speed > Y_MAX_POWER) speed = Y_MAX_POWER;
            else if (speed < -Y_MAX_POWER) speed = -Y_MAX_POWER;

            double MOTOR_GAIN = Math.abs(motorVoltage / robot.getVoltage());

            robot.limelight.update();
            updateTelemetry();
            robot.odoDrivetrain.driveRobot(speed * MOTOR_GAIN, 0, 0);
        }
        robot.odoDrivetrain.stopMotor();

        runtime.reset();
        while((robot.limelight.getTx() <= -0.5 || robot.limelight.getTx() >= 0.5) && robot.opMode.opModeIsActive() && robot.limelight.getStat() && runtime.seconds() <= 7){
            double speed = robot.limelight.getTx() * P_MOVE_STEER_GAIN;
            if (speed > -X_START_POWER && speed < 0) speed = -X_START_POWER;
            else if (speed < X_START_POWER && speed > 0) speed = X_START_POWER;
            else if (speed > X_MAX_POWER) speed = X_MAX_POWER;
            else if (speed < -X_MAX_POWER) speed = -X_MAX_POWER;

            double MOTOR_GAIN = Math.abs(motorVoltage / robot.getVoltage());

            robot.limelight.update();
            updateTelemetry();
            robot.odoDrivetrain.driveRobot(0, speed * MOTOR_GAIN, 0);
        }
        robot.odoDrivetrain.stopMotor();

        robot.limelight.update();

        updateTelemetry();

        robot.subsystem.claw.clawturning.setPosition((Claw.calcTurningPos(robot.limelight.getAngle()) - 500.0) / 2000.0);

        robot.waitFor(1000);
        robot.subsystem.claw.down();

        robot.waitFor(1000).subsystem.claw.grab();

        robot.waitFor(750);
        right_IngrabberPut();

        robot.subsystem.claw.clawturning.setPwmDisable();
        return this;
    }



    public Command left_IngrabberEatForAuto(){//TODO:待测试
        robot.syncRun(() -> {
                    robot.subsystem.stretcher.LeftExpandForward();
                },
                () ->{
                    robot.waitFor(100.0);
                    robot.subsystem.ingrabber.left_Eat();
                    robot.waitFor(1000.0);
                    robot.subsystem.ingrabber.left_stop();
                });
        return this;
    }

    public Command left_IngrabberEatForOpMode(){//TODO:待测试
        robot.syncRun(() -> {
                    robot.subsystem.stretcher.LeftExpandForward();
                },
                () ->{
                    robot.waitFor(100.0);
                    robot.subsystem.ingrabber.left_EatPos();
                });
        return this;
    }

    //strecher向后放sample
    public Command left_IngrabberPut(){//TODO待测试
        robot.syncRun(()->{
                    robot.subsystem.stretcher.LeftExpandBack();
                },
                ()->{
                    robot.subsystem.ingrabber.left_Put();
                    robot.waitFor(WaitForPutTime);
                    robot.subsystem.ingrabber.left_stop();
                });
        return this;
    }

    //初始化
    public Command IngrabberInit(){
        robot.syncRun(
                () -> {
                    robot.subsystem.stretcher.right_auto_init();
                    robot.subsystem.stretcher.left_auto_init();
                },
                () -> {
                    robot.waitFor(250.0);
//                    robot.subsystem.ingrabber.right_auto_init();
                    robot.subsystem.ingrabber.left_auto_init();
                }
        );
        return this;
    }

    //用于将样本放在篮子里
    public Command PutterLaySample(){//TODO待测试
        robot.syncRun(()->{
                    robot.waitFor(1000);
                    robot.subsystem.thrower.Lift();},
                ()->{
                    robot.subsystem.arm.ThrowTop(1);}
        );
        return this;
    }

    //用于让放完sample的机械臂回到原
    public Command PutterBackToPosition() {//TODO待测试
        robot.syncRun(() -> {
                    robot.subsystem.thrower.Down();
                    robot.waitFor(100.0);
                    robot.subsystem.thrower.Liftmotor.setPower(0.0);
                },
                () -> {
                    robot.waitFor(100);
                    robot.subsystem.arm.down();
                }
        );
        return this;
    }

    //用于吃已经挂在边框上的specimen
    public Command EatSpecimen(){//TODO:待测试

        robot.syncRun(() -> {
                    robot.subsystem.arm.down();
                    robot.waitFor(500);
                    robot.subsystem.arm.stop();
                    robot.subsystem.grabber.release();
                    robot.waitFor(750);
                    robot.subsystem.grabber.grab();
                },
                ()->{
                    robot.waitFor(2500);
                }
        );
        return this;
    }


    //用于抓起挂在场边的specimen
    public Command grabup(){//TODO待测试
        robot.syncRun(
                ()->{
                    robot.subsystem.grabber.grab();
                },
                ()->{
                    robot.waitFor(220);
                    robot.subsystem.arm.DunkTop(1);
                    robot.waitFor(900);
                    robot.subsystem.arm.stop();
                }
        );
        return this;
    }

    public Command grabup2(){//TODO待测试
        robot.syncRun(
                ()->{
                    robot.subsystem.grabber.grab();
                },
                ()->{
                    robot.waitFor(100);
                    robot.subsystem.arm.DunkTop(1);
                    robot.waitFor(900);
                    robot.subsystem.arm.stop();
                }
        );
        return this;
    }

    //用于抓起挂在场边的specimen
    public Command Littlegrabup(){//TODO待测试
        robot.syncRun(
                ()->{
                    robot.subsystem.grabber.grab();
                },
                ()->{
                    robot.waitFor(500);
                    robot.subsystem.arm.up(0.1);
                    robot.waitFor(500);
                    robot.subsystem.arm.stop();
                }
        );
        return this;
    }

    public static double RELEASE_TIME = 250 ;
    public static double RELEASE_TIME2 = 275;

    //一键扣篮
    public Command SlamFromTop(){
        robot.syncRun(()->{
            robot.subsystem.arm.DunkBottom(0.8);
            robot.waitFor(700);
            robot.subsystem.arm.stop();
        },
                ()->{
            robot.waitFor(RELEASE_TIME);
            robot.subsystem.grabber.release();
                });
        return this;
    }

    public Command SlamFromTop2(){
        robot.syncRun(()->{
                    robot.subsystem.arm.DunkBottom(0.85);
                    robot.waitFor(860);
                    robot.subsystem.arm.stop();
                },
                ()->{
                    robot.waitFor(RELEASE_TIME2);
                    robot.subsystem.grabber.release();
                });
        return this;
    }

    public Command endGame(){
        robot.syncRun(
                () -> {
                    robot.subsystem.arm.EndTop(1);
                    robot.waitFor(750).subsystem.arm.stop();
                },
                () -> {
                    robot.waitFor(1000).subsystem.arm.DunkBottom(0.5);
//                    robot.waitFor(750).subsystem.arm.stop();
                }
        );
        return this;
    }

    public Command turnForSample(){
        robot.limelight.update();
        robot.subsystem.claw.clawturning.setPosition((Claw.calcTurningPos(robot.limelight.getAngle()) - 500.0) / 2000.0);
        return this;
    }
}