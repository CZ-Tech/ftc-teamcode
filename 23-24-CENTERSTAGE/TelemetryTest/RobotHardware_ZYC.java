package org.firstinspires.ftc.teamcode.TelemetryTest;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.opmode.route.Route;
import org.firstinspires.ftc.teamcode.utlity.FileUtil;

public class RobotHardware_ZYC {
    private IMU imu = null;      // Control/Expansion Hub IMU
    public final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    private final Telemetry telemetry;
    public Route_ZYC route;

    public RobotHardware_ZYC(LinearOpMode opmode) {
        myOpMode = opmode;
        telemetry = opmode.telemetry;
        route = new Route_ZYC(this);
        //vision = new Vision(this);
        //file = new FileUtil();
    }

    public void init() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();


    }

    public RobotHardware_ZYC sleep(long milliseconds) {
        myOpMode.sleep(milliseconds);
        return this;
    }

    public RobotHardware_ZYC syncrun(Runnable... functions) {
        for (Runnable function : functions) {
            new Thread(function).start();
        }
        return this;
    }

}
