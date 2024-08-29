package org.firstinspires.ftc.teamcode.test;

//import com.acmerobotics.dashboard.telemetry/.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
//import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FtcMenu", group = "Robot")
public class FtcMenu extends LinearOpMode {

    @Override
    public void runOpMode() {
        List<String> menu = new ArrayList<>();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        menu.add(0, "aaa");
        menu.add(1, "aaa");
        menu.add(2, "aaa");
        menu.add(3, "aaa");
        menu.add(4, "aaa");
        menu.add(5, "aaa");
        boolean thisDup;
        boolean thisDdown;
        boolean lastDup=false;
        boolean lastDdown=false;
        int currentSelected = 1;
        int menuSize = menu.size();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        while (opModeInInit() || opModeIsActive()) {
            thisDup= gamepad1.dpad_up;
            thisDdown=gamepad1.dpad_down;
            if(thisDup && !lastDup)
                currentSelected= Range.clip(currentSelected-1,0,5);
            if(thisDdown && !lastDdown)
                currentSelected= Range.clip(currentSelected+1,0,5);;
            for (int i = 0; i < menuSize; i++) {
                telemetry.addData(String.valueOf(i), currentSelected == i ? "<font color=red>" + menu.get(i) + "</font>" : menu.get(i));
            }
            lastDup=thisDup;
            lastDdown=thisDdown;
            telemetry.update();
        }

    }
}

