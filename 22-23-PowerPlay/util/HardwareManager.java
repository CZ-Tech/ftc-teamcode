package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//这个类对原来单独对硬件进行的get操作整合到了这个类
//目的：方便查看&方便对硬件更名（所有硬件名字的更改均在HardwareManager.HardwareName中）
//使用：HardwareManager manager = new HardwareManager(hardwareMap);
//     DcMotor motor = manager.getMotor(Motor.m0);
//注意执行new的时候应当在runOpMode()里执行，不然会丢出NullPointerException
//原因：在执行runOpMode()前FTC程序会自动初始化hardwareMap，在之前调用会调用过来一个null
//详见：OpMode.java Line:70
public class HardwareManager {
    private final HardwareMap map;

    /*
     *这玩意应当在runOpMode()里执行
     */
    public HardwareManager(HardwareMap hardwareMap) {
        this.map = hardwareMap;
    }

    public DcMotorEx getMotor(Motor motor) {
        return map.get(DcMotorEx.class, motor.name);
    }

    public BNO055IMU getIMU() {
        return map.get(BNO055IMU.class, "imu");
    }

    public enum Motor {
        //马达的枚举类
        Motor0(HardWareName.Motor0),
        Motor1(HardWareName.Motor1),
        Motor2(HardWareName.Motor2),
        Motor3(HardWareName.Motor3),
        LFmotor(HardWareName.LFmotor),
        LRmotor(HardWareName.LRmotor),
        RFmotor(HardWareName.RFmotor),
        RRmotor(HardWareName.RRmotor);
        String name;

        Motor(String name) {
            this.name = name;
        }
    }

    private static class HardWareName {
        //马达的名字
        public static final String Motor0 = "slider";//滑轨的马达
        public static final String Motor1 = "duck";//转盘的马达
        public static final String Motor2 = "left1";
        public static final String Motor3 = "right1";
        //麦轮代码
        // 0 左前 lfmotor
        // 1 右前 rfmotor
        // 2 右后 rrmotor
        // 3 左后 lrmotor
        public static final String LFmotor = "lfmotor";
        public static final String RFmotor = "rfmotor";
        public static final String RRmotor = "rrmotor";
        public static final String LRmotor = "lrmotor";
    }
}
