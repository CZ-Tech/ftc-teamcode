package org.firstinspires.ftc.teamcode.common.drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.Robot;

public class GyroTracker {
    private double initAngle;
    private double totalAngle;
    public Robot robot;

    public GyroTracker(Robot robot){
        this.robot = robot;
    }

    public void init(double initAngle){
        this.initAngle = initAngle;
        this.totalAngle = 0;
    }

    /**
     * 角度归一化到 [-180, 180] 范围
     */
    private static double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * 获取当前θ相对于陀螺仪示数的值
     */
    public double getCurrentOffset(AngleUnit angleUnit){
        double currentU = robot.odoDrivetrain.getHeading(angleUnit);

        double beta = initAngle - totalAngle - currentU;
        return normalizeAngle(beta);
    }

    /**
     * 处理陀螺仪重置事件
     * 必须在陀螺仪实际重置前调用
     */
    public void handleReset(AngleUnit angleUnit){
        totalAngle += robot.odoDrivetrain.getHeading(angleUnit);
    }
}
