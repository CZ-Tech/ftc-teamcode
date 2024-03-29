package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "自动操控模式_test", group = "Robot", preselectTeleOp = "手动阶段_Test")
public class AutoOp19656_Test extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() {
        robot.init();
        robot.initDoubleVision();
        telemetry.addData("Status", "Initialized");
        waitForStart();
        robot.resetYaw();

        //   A
        // d G b
        //   V
        robot
            .A(24)  // 向前
            .V(2)   // 退后
            .sleep(500)
            .G(0)   // 自转到0度
            .G(90)  //自转到180度
            .A(12)  //向前12
            .b(84)  //向右平移至后台
            .V(18)  //向前至板前
            .b(5)   //靠紧板
        ;

        telemetry.addData("Path", "Complete");
        telemetry.update();

        robot.closeVision();
    }
}
