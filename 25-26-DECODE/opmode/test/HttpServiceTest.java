package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;

@TeleOp(name = "HttpServiceTest", group = "Test")
public class HttpServiceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("HTTP JSON 服务已在后台运行");
        telemetry.addLine("监听端口: 8888");
        telemetry.addLine("API:");
        telemetry.addLine("  POST /              — 发送 JSON");
        telemetry.addLine("  GET  /              — 查看当前");
        telemetry.addLine("  POST /save/{name}   — 持久化到路径");
        telemetry.addLine("  GET  /{name}        — 查看路径");
        telemetry.addLine("  POST /load/{name}   — 加载路径");
        telemetry.addLine("  POST /clear/{name}  — 删除路径");
        telemetry.addLine("  GET  /list          — 列出所有路径");
        telemetry.addLine("  GET  /position      — 获取机器人当前场地坐标");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("当前 JSON", HttpJsonService.getCurrentJson());
            telemetry.addData("已保存路径", HttpJsonService.getPathNames().toString());
            telemetry.update();
            idle();
        }
    }
}
