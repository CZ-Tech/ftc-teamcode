package org.firstinspires.ftc.teamcode.sh;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.WebServer;
import com.qualcomm.robotcore.wifi.NetworkType;

import org.firstinspires.ftc.robotserver.internal.webserver.CoreRobotWebServer;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

@TeleOp(name = "WebServerTest", group = "SH")

public class WebServerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CoreRobotWebServer server = new CoreRobotWebServer(5436, NetworkType.WIFIDIRECT);
        server.start();

        NanoHTTPD nanoHTTPD = new NanoHTTPD(6576) {
            @Override
            public synchronized void closeAllConnections() {
                super.closeAllConnections();
            }
        };
        try {
            nanoHTTPD.start();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }



    }
}
