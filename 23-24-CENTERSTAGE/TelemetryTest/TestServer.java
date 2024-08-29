package org.firstinspires.ftc.teamcode.TelemetryTest;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.*;
import java.net.*;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Server", group = "Test")
public class TestServer extends LinearOpMode{

    RobotHardware_ZYC robot = new RobotHardware_ZYC(this);

    public void runOpMode() {
        // 创建ServerSocket并指定端口
        robot.init();
        waitForStart();
        ServerSocket serverSocket = null;
        try {
            serverSocket = new ServerSocket(1145, 0, InetAddress.getByName("127.0.0.1"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // 监听并接受客户端的连接
        Socket clientSocket = null;
        try {
            clientSocket = serverSocket.accept();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // 读取客户端发送的HTTP请求
        BufferedReader in = null;
        try {
            in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        String inputLine;
        while (true) {
            try {
                if (!((inputLine = in.readLine()) != null)) break;
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
            telemetry.addData("Http:", inputLine);
            telemetry.update();
        }

        // 向客户端发送HTTP响应
        PrintWriter out = null;
        try {
            out = new PrintWriter(clientSocket.getOutputStream(), true);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        out.println("HTTP/1.1 200 OK");
        out.println("Content-Type: text/plain");
        out.println("Connection: close");
        out.println();
        out.println("Hello, client!");

        // 关闭连接
        try {
            in.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        out.close();
        try {
            clientSocket.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            serverSocket.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }
}
