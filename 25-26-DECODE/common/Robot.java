package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.command.AutoAimController;
import org.firstinspires.ftc.teamcode.common.command.Command;
import org.firstinspires.ftc.teamcode.common.command.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.drive.GyroTracker;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.drive.OdoDrivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.AutoTask;
import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;
import org.firstinspires.ftc.teamcode.common.util.Pattern;

import org.firstinspires.ftc.teamcode.common.vision.Vision;
import org.firstinspires.ftc.teamcode.common.vision.VisionC270;
import org.firstinspires.ftc.teamcode.common.vision.VisionLimelight;


public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Vision vision;
    public LinearOpMode opMode;
    public OdoDrivetrain odoDrivetrain;
    public Subsystem subsystem;
    public Command command;
    public GamepadEx gamepad1;
    public GamepadEx gamepad2;
    public Alliance teamColor;
    public OpModeState opModeState;
//    public GoBildaPinpointDriver odo;
    public MixedOdo odo;
    public Limelight3A limelight;
    public VisionLimelight visionLimelight;
    public GyroTracker gyroTracker;
    public Pattern pattern;
    public VisionC270 visionC270;
    public AutoAimController autoAimController = new AutoAimController();
//    public LLMegaTag2 llPos;
//    public IMU imu;

    /**
     * 初始化机器人
     *
     * @param opMode 在 opMode 里通过 robot.init(this); 进行初始化。
     */
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap; //硬件映射
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());//DH上的遥测，使用了FTCDashboard
//        this.imu = hardwareMap.get(IMU.class, ImuName);
//        this.limelight = hardwareMap.get(Limelight3A.class, Globals.limelight);
//        this.limelight.setPollRateHz(20); // 这设置了我们向Limelight请求数据的频率（每秒100次）
//        limelight.pipelineSwitch(6);
//        limelight.start();

//        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, Globals.odoName);
        this.odo = new MixedOdo(this);

        HttpJsonService.setActiveRobot(this);

        this.vision = new Vision(this); //视觉模块
//        this.visionLimelight = new VisionLimelight(this);
        this.subsystem = new Subsystem(this); //上层子系统
        this.gamepad1 = new GamepadEx(opMode.gamepad1); //一个控制器
        this.gamepad2 = new GamepadEx(opMode.gamepad2); //另一个控制器
        this.command = new Command(this); //命令系统

        this.odoDrivetrain = new OdoDrivetrain(this);

        this.gyroTracker = new GyroTracker(this);



//        llPos = new LLMegaTag2(this);

//        this.visionC270 = new VisionC270(this);





//        this.imu = hardwareMap.get(IMU.class, "imu");
//        imu.resetYaw();



    }

    /**
     * 获取当前电压
     *
     * @return voltage
     */
    public double getVoltage() {
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    /**
     * 以多线程运行函数
     *
     * @param commands 需要同时运行的命令（函数或者方法）
     * @return Robot
     */
    public Robot syncRun(Runnable... commands) {
        for (Runnable command : commands) {
            new Thread(command).start();
        }
        return this;
    }


    /**
     * 等待一段时间，并会在opmode停止时自动结束避免报错
     * @param millisecond
     * @return 甚至还能链式调用
     */
    public Robot waitFor(double millisecond){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= millisecond && opMode.opModeIsActive());
        return this;
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted (which usually
     * indicates that the OpMode has been stopped).
     * <p>This is simple shorthand for {@link Thread#sleep(long) sleep()}, but it does not throw {@link InterruptedException}.</p>
     *
     * @param millis amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
