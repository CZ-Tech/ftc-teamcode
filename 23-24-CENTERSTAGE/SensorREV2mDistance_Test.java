package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "距离传感器测试", group = "A")
public abstract class SensorREV2mDistance_Test extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        DistanceSensor sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        while (opModeIsActive()) {
            telemetry.addData("距离", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("距离", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("距离", String.format("%.01f meter", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("距离", String.format("%.01f inch", sensorDistance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
