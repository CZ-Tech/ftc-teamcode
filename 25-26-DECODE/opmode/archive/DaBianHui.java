package org.firstinspires.ftc.teamcode.opmode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="答辩会专用")
public class DaBianHui extends LinearOpMode {
    ServoImplEx north, south, east, west;

    //这两个自己改数值
    double startPos = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        //改成ds上的名称
        south = hardwareMap.get(ServoImplEx.class, "n");
        north = hardwareMap.get(ServoImplEx.class, "s");
        east = hardwareMap.get(ServoImplEx.class, "e");
        west = hardwareMap.get(ServoImplEx.class, "w");

        south.setPwmRange((new PwmControl.PwmRange(500, 2500)));
        north.setPwmRange(new PwmControl.PwmRange(500, 2500));
        west.setPwmRange((new PwmControl.PwmRange(500, 2500)));
        east.setPwmRange(new PwmControl.PwmRange(500, 2500));


        //设置初始位置
        initServo();

        telemetry.addLine("Ready");

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();

        //等待结束
        while(opModeIsActive()){
            telemetry.addData("Position S N E W", new double[]{
                    south.getPosition() * 2000 + 500,
                    north.getPosition() * 2000 + 500,
                    east.getPosition() * 2000 + 500,
                    west.getPosition() * 2000 + 500
            });


            runtime.reset();
            while (runtime.milliseconds() <= 3500){
                south.setPosition(south.getPosition() + 0.01);
                north.setPosition(north.getPosition() + 0.01);
                east.setPosition(east.getPosition() + 0.01);
                west.setPosition(west.getPosition() + 0.01);
                sleep(200);
            }

            runtime.reset();
            while (runtime.milliseconds() <= 3500){
                south.setPosition(south.getPosition() - 0.01);
                north.setPosition(north.getPosition() - 0.01);
                east.setPosition(east.getPosition() - 0.01);
                west.setPosition(west.getPosition() - 0.01);
                sleep(200);
            }

            runtime.reset();
            while (runtime.milliseconds() <= 3500){
                south.setPosition(south.getPosition() - 0.01);
                north.setPosition(north.getPosition() - 0.01);
                east.setPosition(east.getPosition() - 0.01);
                west.setPosition(west.getPosition() - 0.01);
                sleep(200);
            }

            runtime.reset();
            while (runtime.milliseconds() <= 3500){
                south.setPosition(south.getPosition() + 0.01);
                north.setPosition(north.getPosition() + 0.01);
                east.setPosition(east.getPosition() + 0.01);
                west.setPosition(west.getPosition() + 0.01);
                sleep(200);
            }

            telemetry.update();

        }
    }

    public void initServo(){
        south.setPosition((startPos - 500.0) / 2000.0);
        north.setPosition((startPos - 500.0) / 2000.0);
        east.setPosition((startPos - 500.0) / 2000.0);
        west.setPosition((startPos - 500.0) / 2000.0);
    }
}
