package org.firstinspires.ftc.teamcode.opmode.route;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;


public class Route {
    private final RobotHardware robot;
//    private final RobotHardware_ZYC robot_zyc;
    public Route(RobotHardware robot) {
        this.robot = robot;
    }

    public void r11() { R11.run(robot); }
    public void r12() {
        R12.run(robot);
    }
    public void r13() {
        R13.run(robot);
    }
    public void r21() {
        R21.run(robot);
    }
    public void r22() {
        R22.run(robot);
    }
    public void r23() {
        R23.run(robot);
    }
    public void b11() {
        B11.run(robot);
    }
    public void b12() {
        B12.run(robot);
    }
    public void b13(ArrayList<String[]> args) {
        B13.run(robot);
    }
    public void b21() {
        B21.run(robot);
    }
    public void b22() {
        B22.run(robot);
    }
    public void b23() {
        B23.run(robot);
    }
    //回后台
    public void goBackStage(double distance) {
        robot
            .s(1)  // 后退
            .d(distance)   // 平移
            .w(12) // 进坑
        ;
    }
}
