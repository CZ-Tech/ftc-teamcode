package org.firstinspires.ftc.teamcode.TelemetryTest;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;
import java.util.ArrayList;

@TeleOp(name = "Config", group = "Test")
public class Config extends LinearOpMode {
    //RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();
    private double mul = 1;
    private int pages = 1;
    private  int selected_page = pages;
    private  int selected = 0;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ArrayList<String[]> myargs = new ArrayList<String[]>();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
//        robot.resetYaw();
        Properties prop = new Properties();
        try {
            // 加载配置文件
            prop.load(new FileInputStream("/sdcard/FIRST/tflitemodels/args.properties"));
            for (String key : prop.stringPropertyNames()) {
                myargs.add(new String[] {key, prop.getProperty(key, "0")});
            }
            if (myargs.size() % 10 != 0) {
                int j = myargs.size() % 10;
                for (int i = 0; i <= j;  i++)
                    myargs.add(new String[] {"-", "-"});
            }
            pages = myargs.size() / 10;
            telemetry.addData("Load properties successfully", null);
            telemetry.addData("十字键上/下选择，左/右更改数值，Y/A翻页", null);
            telemetry.update();
        } catch (IOException e) {
            telemetry.addData("Failed to load properties", null);
            telemetry.update();
        }
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        waitForStart();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            telemetry.addData("当前方差值(按左/右肩键十倍修改)", mul);

            for (int i = 0; i < 10 ; i++) {
                telemetry.addData(selected == i ? "<font color=green>" + myargs.get((selected_page - 1) * 10 + i)[0] + "</font>" : myargs.get((selected_page - 1) * 10 + i)[0], myargs.get((selected_page - 1) * 10 + i)[1]);
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                mul = mul / 10;
                robot.sleep(50);
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                mul = mul * 10;
                robot.sleep(50);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                ++selected;
                if (selected > 9) {
                    selected = 0;
                }
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                --selected;
                if (selected < 0) {
                    selected = 9;
                }
            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                --selected_page;
                if (selected_page < 1) {
                    selected_page = pages;
                }
            }
            if (currentGamepad1.a && !previousGamepad1.a) {
                ++selected_page;
                if (selected_page > pages) {
                    selected_page = 1;
                }
            }
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                myargs.get((selected_page - 1) * 10 + selected)[1] = Double.toString(Double.parseDouble(myargs.get((selected_page - 1) * 10 + selected)[1]) + mul);
                robot.sleep(50);
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                myargs.get((selected_page - 1) * 10 + selected)[1] = Double.toString(Double.parseDouble(myargs.get((selected_page - 1) * 10 + selected)[1]) - mul);
                robot.sleep(50);
            }
//            telemetry.addData("十字键上/下", "光标向上/向下");
//            telemetry.addData("十字键左右", "");
            telemetry.update();
        }
        for (String[] data : myargs) {
            prop.setProperty(data[0], data[1]);
        }
        try {
            prop.store(new FileOutputStream("/sdcard/FIRST/tflitemodels/test.properties"), null);
            telemetry.addData("Store data successfully!", null);
            telemetry.update();
            robot.sleep(1500);
        } catch (IOException e) {
            telemetry.addData("Fail to store data!", null);
            telemetry.update();
            robot.sleep(1500);
        }


    }
}
