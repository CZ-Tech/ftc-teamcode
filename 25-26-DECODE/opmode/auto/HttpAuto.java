package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.command.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.command.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;
import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;

/**
 * 空白等待自动 OpMode，初始化所有硬件后进入空闲循环，
 * 通过 {@link HttpJsonService} 的 HTTP 端点接受路径执行命令。
 *
 * <p>该 OpMode 自身不执行任何轨迹，所有路径执行均通过 HTTP API 触发：</p>
 * <ul>
 *   <li>{@code POST /run/saved/{pathName}} — 执行已保存的路径</li>
 *   <li>{@code POST /run/temp} — 执行请求体中的临时路径（不保存）</li>
 * </ul>
 *
 * <h3>状态机</h3>
 * <pre>
 *   初始化硬件 → waitForStart → executionReady=true
 *     → 空闲循环 [检查 pendingExecution → 执行路径 → 重复]
 *     → 停止时: executionReady=false, stopAndClearAll()
 * </pre>
 */
@Autonomous(name = "HttpAuto", group = "HTTP")
public class HttpAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // --- 阶段 1: 硬件初始化（照抄 JsonPathOpMode） ---
        Robot robot = new Robot();
        robot.init(this);

        // 扫描对象树使 @AutoTask 命令可用，并设置 OpMode 名称以便 AzConductor 检测
        HttpJsonService.scanObjectTree(robot);
        HttpJsonService.setActiveOpModeName("HttpAuto");

        MixedOdo.isPoseInitialized = true;

        PinpointTrajectory trajectory = new PinpointTrajectory(robot);

        telemetry.addData("Status", "已初始化，等待开始");
        telemetry.update();

        // --- 阶段 2: 等待开始按钮 ---
        waitForStart();

        if (!opModeIsActive()) {
            TaskLoopFrame.stopAndClearAll();
            return;
        }

        // --- 阶段 3: 空闲循环，接受 HTTP 路径执行命令 ---
        HttpJsonService.setExecutionReady(true);

        telemetry.addData("Status", "就绪，等待 HTTP 路径命令 (端口 8888)");
        telemetry.update();

        while (opModeIsActive()) {
            if (HttpJsonService.hasPendingExecution()) {
                String json = HttpJsonService.getPendingPathJson();
                HttpJsonService.clearPendingExecution();

                if (json != null && !json.isEmpty()) {
                    telemetry.addData("Status", "正在执行路径...");
                    telemetry.update();

                    try {
                        // TrajectoryLoader.execute() 会阻塞直到路径完成
                        // 或者在 opModeIsActive() 返回 false 时退出
                        new TrajectoryLoader(trajectory).execute(json);
                    } catch (Exception e) {
                        telemetry.addData("Error", e.getMessage());
                        telemetry.update();
                    }

                    // 路径完成后停止残余运动
                    robot.odoDrivetrain.driveRobotFieldCentric(0, 0, 0);

                    // 等待异步任务（射击、传送等）完成
                    TaskLoopFrame.joinAllTask(30000);

                    telemetry.addData("Status", "就绪，等待 HTTP 路径命令 (端口 8888)");
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
