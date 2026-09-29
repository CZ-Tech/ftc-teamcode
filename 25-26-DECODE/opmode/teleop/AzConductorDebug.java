package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.command.SplineTracker;
import org.firstinspires.ftc.teamcode.common.command.SplineTrajectoryLoader;
// 旧方案（时间驱动 P 控制器）：
// import org.firstinspires.ftc.teamcode.common.command.PinpointTrajectory;
// import org.firstinspires.ftc.teamcode.common.command.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;

/**
 * 专用于 AzConductor 的调试 OpMode（TeleOp 模式，无 30 秒时间限制）。
 * 初始化所有硬件后进入空闲循环，通过 {@link HttpJsonService} 的 HTTP 端点接受路径执行命令。
 *
 * <p>使用 {@link SplineTracker}（位置驱动）进行路径跟踪，
 * 通过 {@link SplineTrajectoryLoader} 兼容 AzConductor 的 JSON 格式。</p>
 *
 * <p>与 {@code HttpAuto} 的区别：</p>
 * <ul>
 *   <li>使用 {@code @TeleOp} 而非 {@code @Autonomous} —— 无时间限制</li>
 *   <li>在 {@code waitForStart()} 之前即调用 {@code scanObjectTree()} 使命令立即可用</li>
 *   <li>设置 {@code activeOpModeName} 以便 AzConductor 自动检测</li>
 * </ul>
 *
 * <h3>HTTP API（通过已运行的 HttpJsonService）</h3>
 * <ul>
 *   <li>{@code GET  /status}              — 查询 OpMode 状态</li>
 *   <li>{@code GET  /position}            — 查询机器人当前位置</li>
 *   <li>{@code POST /run/saved/{pathName}} — 执行已保存的路径</li>
 *   <li>{@code POST /run/temp}            — 执行请求体中的临时路径</li>
 *   <li>{@code POST /commands/run/{name}} — 调用单个 @AutoTask 命令</li>
 * </ul>
 *
 * <h3>状态机</h3>
 * <pre>
 *   初始化硬件 → scanObjectTree → executionReady=true
 *     → waitForStart → 空闲循环 [检查 pendingExecution → 执行路径 → 重复]
 *     → 停止时: executionReady=false, activeOpModeName=null, stopAndClearAll()
 * </pre>
 */
@TeleOp(name = "AzConductor调试", group = "调试")
public class AzConductorDebug extends LinearOpMode {

    @Override
    public void runOpMode() {
        // --- 阶段 1: 硬件初始化 ---
        Robot robot = new Robot();
        robot.init(this);

        // 立即扫描对象树，使所有 @AutoTask 命令变为 ready 状态
        HttpJsonService.scanObjectTree(robot);

        // 设置 OpMode 名称，以便 AzConductor 通过 GET /status 自动检测
        HttpJsonService.setActiveOpModeName("AzConductorDebug");

        // 标记就绪 —— 此时 GET /status 即可返回 executionReady=true
        HttpJsonService.setExecutionReady(true);

        MixedOdo.isPoseInitialized = true;

        // 新方案（位置驱动，牛顿法最近点 + 加速度前馈）：
        SplineTracker tracker = new SplineTracker(robot);
        // 旧方案（时间驱动 P 控制器）：
        // PinpointTrajectory trajectory = new PinpointTrajectory(robot);

        telemetry.addData("Status", "已初始化，等待开始");
        telemetry.addData("OpMode", "AzConductor调试");
        telemetry.addData("HTTP端口", "8888");
        telemetry.update();

        // --- 阶段 2: 等待开始按钮 ---
        waitForStart();

        if (!opModeIsActive()) {
            HttpJsonService.setExecutionReady(false);
            HttpJsonService.setActiveOpModeName(null);
            TaskLoopFrame.stopAndClearAll();
            return;
        }

        // --- 阶段 3: 空闲循环，接受 HTTP 路径执行命令 ---
        telemetry.addData("Status", "就绪，等待 HTTP 路径命令 (端口 8888)");
        telemetry.addData("可用端点", "/status, /position, /run/saved/*, /run/temp, /commands/run/*");
        telemetry.update();

        while (opModeIsActive()) {
            if (HttpJsonService.hasPendingExecution()) {
                String json = HttpJsonService.getPendingPathJson();
                HttpJsonService.clearPendingExecution();

                if (json != null && !json.isEmpty()) {
                    telemetry.addData("Status", "正在执行路径...");
                    telemetry.update();

                    Log.i("auto", json);

                    try {
                        // 新方案：SplineTrajectoryLoader.execute()
                        new SplineTrajectoryLoader(tracker).execute(json);
                        // 旧方案：
                        // new TrajectoryLoader(trajectory).execute(json);
                    } catch (Exception e) {
                        telemetry.addData("Error", e.getMessage());
                        telemetry.update();
                    }

                    // 路径完成后停止残余运动
                    robot.odoDrivetrain.driveRobotFieldCentric(0, 0, 0);

                    // 等待残留异步任务完成（默认已阻塞执行，仅 ASYNC_TASKS=true 时才有残余）
                    TaskLoopFrame.joinAllTask(5000);

                    telemetry.addData("Status", "就绪，等待 HTTP 路径命令 (端口 8888)");
                    telemetry.addData("可用端点", "/status, /position, /run/saved/*, /run/temp, /commands/run/*");
                    telemetry.update();
                }
            }

            // 休眠 50ms，避免忙等待
            Robot.sleep(50);
        }

        // --- 阶段 4: 停止 ---
        HttpJsonService.setExecutionReady(false);
        HttpJsonService.setActiveOpModeName(null);
        TaskLoopFrame.stopAndClearAll();
    }
}
