package org.firstinspires.ftc.teamcode.TelemetryTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.LoadProperties;

import java.util.HashMap;

@TeleOp(name = "调参测试", group = "Test")
public class LoadTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();
    private HashMap<String, Double> myargs;

    @Override
    public void runOpMode() {
        robot.init();
        robot.resetYaw();
        LoadProperties prop = new LoadProperties("/sdcard/FIRST/tflitemodels/args.properties", "B13");
        try {
            myargs = prop.load();
        } catch (Throwable e) {
            throw new RuntimeException(e);
        }
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        waitForStart();
        telemetry.addData("w1.distance", myargs.get("w1.distance"));
        telemetry.addData("s.distance", myargs.get("s.distance"));
        telemetry.addData(null,
                "<html>\n" +
                        "<head>\n" +
                        "    <title>显示图片示例</title>\n" +
                        "</head>\n" +
                        "<body>\n" +
                        "\n" +
                        "<h1>这是一张先辈的图片</h1>\n" +
                        "<img src=\"/sdcard/FIRST/tflitemodels/homo.jpg\" alt=\"一只可爱的先辈\">\n" +
                        "\n" +
                        "</body>\n" +
                        "</html>"
                );
        telemetry.update();
        while (opModeIsActive()) {
            robot.sleep(500);
        }
    }
}
