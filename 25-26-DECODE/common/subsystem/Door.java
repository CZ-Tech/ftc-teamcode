package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

@Config
public class Door {
    public Robot robot;

    public CRServoImplEx doorWheel;
//    public ServoImplEx doorController;

    public static double DOOR_END = 1200;    //在上面
    public static double DOOR_STANDBY = 1980;//推出去

    public Door(Robot robot){
        this.robot = robot;
//        doorController = robot.hardwareMap.get(ServoImplEx.class, Globals.doorController);
        doorWheel = robot.hardwareMap.get(CRServoImplEx.class, Globals.doorWheel);

        doorWheel.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        doorController.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public Door spin(double power){
        doorWheel.setPower(power);
        return this;
    }

    public Door spin(){
        return this.spin(1);
    }

    public Door stop(){
        return this.spin(0);
    }

    public Door standBy(){
//        doorController.setPosition((DOOR_STANDBY - 500.0) / 2000.0);
        return this;
    }

    public Door end(){
//        doorController.setPosition((DOOR_END - 500.0) / 2000.0);
        return this;
    }
}
