package org.firstinspires.ftc.teamcode.TelemetryTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;

@Autonomous(name = "IOTest", group = "Test")
public class IOTest extends LinearOpMode {
    RobotHardware_ZYC robot = new RobotHardware_ZYC(this);
    private ElapsedTime runtime = new ElapsedTime();
    private int test1, test2, test3 = 0;

//    private String ;

    @Override
    public void runOpMode() {
        robot.init();
        //robot.resetYaw();
        Properties prop = new Properties();
        try {
            // 加载配置文件
            prop.load(new FileInputStream("/sdcard/FIRST/tflitemodels/test.properties"));

            // 获取配置项的值
            test1 = Integer.parseInt(prop.getProperty("test1"));
            test2 = Integer.parseInt(prop.getProperty("test2"));
            test3 = Integer.parseInt(prop.getProperty("test3"));

            telemetry.addData("Load properties successfully", null);
            telemetry.update();
        } catch (IOException e) {
            telemetry.addData("Failed to load properties", null);
            telemetry.update();
        }
        waitForStart();
        test1++;
        test2++;
        test3++;
        telemetry.addData("test1", test1);
        telemetry.addData("test2", test2);
        telemetry.addData("test3", test3);
        telemetry.update();

        prop.setProperty("test1", Integer.toString(test1));
        prop.setProperty("test2", Integer.toString(test2));
        prop.setProperty("test3", Integer.toString(test3));

        try {
            prop.store(new FileOutputStream("/sdcard/FIRST/tflitemodels/test.properties"), null);
        } catch (IOException e) {
            telemetry.addData("Fail to store data!", null);
            telemetry.update();
            robot.sleep(1500);
        }


    }
}
