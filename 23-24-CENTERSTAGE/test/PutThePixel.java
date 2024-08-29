package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.*;
import org.firstinspires.ftc.teamcode.utlity.RobotConstants.IntakeArmPosition;


public class PutThePixel {
    public static void putThePixel(RobotHardware robot) {
        putThePixel(robot,IntakeArmPosition.AUTO_BOARD);
    }
    public static void putThePixel(RobotHardware robot, IntakeArmPosition position) {
        robot
                .setIntakeFrontPosition(IntakeFrontPosition.IN)
            .setIntakeBackPosition(IntakeBackPosition.IN)
            .setIntakeArmPosition(position)
            .sleep(1500)
            .setIntakeFrontPosition(IntakeFrontPosition.OUT)
            .setIntakeBackPosition(IntakeBackPosition.OUT)
            .sleep(500)
            .setIntakeArmPosition(IntakeArmPosition.BACK)
        ;
    }
}
