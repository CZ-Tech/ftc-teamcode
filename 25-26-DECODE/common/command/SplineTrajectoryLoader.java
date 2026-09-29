package org.firstinspires.ftc.teamcode.common.command;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.util.HttpJsonService;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.HashMap;
import java.util.Map;

/**
 * SplineTracker 专用的轨迹加载器。
 *
 * 与 {@link TrajectoryLoader} 的区别：
 * - 不解析 duration（SplineTracker 是位置驱动，自动走到终点）
 * - 支持两种关键帧：路径点（有 x,y）和等待点（有 wait）
 * - marker 机制与原有相同
 * - 兼容 AzConductor 的 delayAfterArrive 和 command/commandParams 字段
 */
public class SplineTrajectoryLoader {

    private final SplineTracker tracker;
    private final Map<String, Runnable> markerTasks = new HashMap<>();

    public SplineTrajectoryLoader(SplineTracker tracker) {
        this.tracker = tracker;
    }

    /**
     * 注册标记任务。
     * @param marker JSON 中的 marker 名称
     * @param task   到达该标记时执行的任务
     * @return this（链式调用）
     */
    public SplineTrajectoryLoader addMarkerTask(String marker, Runnable task) {
        markerTasks.put(marker, task);
        return this;
    }

    /**
     * 解析 JSON 并驱动 SplineTracker。
     *
     * JSON 格式示例：
     * <pre>
     * [
     *     {"x": 0,  "y": 0,  "dx": 0, "dy": 0, "heading": 0},
     *     {"x": 24, "y": 0,  "dx": 0, "dy": 0, "heading": 0, "marker": "shoot"},
     *     {"wait": 1.5},
     *     {"x": 48, "y": 24, "dx": 0, "dy": 0, "heading": 90},
     *     {"wait": 2.0, "marker": "done"}
     * ]
     * </pre>
     *
     * @param jsonString 轨迹 JSON 字符串
     */
    public void execute(String jsonString) {
        HttpJsonService.scanObjectTree(tracker.getRobot());
        try {
            JSONArray jsonArray = new JSONArray(jsonString);
            if (jsonArray.length() == 0) return;

            boolean isFirstWaypoint = true;

            for (int i = 0; i < jsonArray.length(); i++) {
                JSONObject obj = jsonArray.getJSONObject(i);

                if (obj.has("wait")) {
                    // --- 等待点 ---
                    double waitSeconds = obj.getDouble("wait");
                    String rawMarker = obj.optString("marker", null);
                    String marker = (rawMarker != null && !rawMarker.isEmpty() && !"null".equals(rawMarker)) ? rawMarker : null;
                    Runnable task = (marker != null) ? markerTasks.get(marker) : null;

                    if (task != null) {
                        Log.i("SplineAuto", marker);
                        tracker.addTime(waitSeconds, task);
                    } else {
                        if (marker != null) {
                            Log.w("SplineAuto", "[BRK3] wait[" + i + "]: marker='" + marker + "' unregistered → task=null, hold without callback");
                        }
                        tracker.addTime(waitSeconds);
                    }

                } else {
                    // --- 路径点 ---
                    double x = obj.getDouble("x");
                    double y = obj.getDouble("y");
                    double dx = obj.optDouble("dx", 0);
                    double dy = obj.optDouble("dy", 0);
                    double heading = obj.optDouble("heading", 0);
                    double delayAfterArrive = obj.optDouble("delayAfterArrive", 0);
                    String rawMarker = obj.optString("marker", null);
                    String marker = (rawMarker != null && !rawMarker.isEmpty() && !"null".equals(rawMarker)) ? rawMarker : null;
                    String command = obj.optString("command", null);
                    JSONArray cpArr = obj.optJSONArray("commandParams");
                    String[] commandParams;
                    if (cpArr != null) {
                        commandParams = new String[cpArr.length()];
                        for (int j = 0; j < cpArr.length(); j++) {
                            commandParams[j] = cpArr.optString(j, "");
                        }
                    } else {
                        commandParams = new String[0];
                    }

                    // 优先级：marker → command
                    Runnable task = (marker != null) ? markerTasks.get(marker) : null;
                    if (marker != null && !marker.isEmpty() && task == null) {
                        Log.w("SplineAuto", "[BRK3] waypoint[" + i + "]: marker='" + marker + "' not in markerTasks (size=" + markerTasks.size() + "), fallback to command");
                    }
                    if (task == null && command != null && !command.isEmpty()) {
                        task = resolveCommandTask(command, commandParams);
                    }

                    // 诊断日志：task 为 null 的原因
                    if (task == null) {
                        if (command != null && !command.isEmpty()) {
                            Log.w("SplineAuto", "[BRK1] waypoint[" + i + "]: command='" + command + "' unresolved → task=null, robot moves but command WONT run");
                        } else if (marker != null && !marker.isEmpty()) {
                            Log.w("SplineAuto", "[BRK3] waypoint[" + i + "]: marker='" + marker + "' unregistered & no command → task=null");
                        } else {
                            Log.d("SplineAuto", "[INFO] waypoint[" + i + "]: plain waypoint, no marker/command");
                        }
                    }

                    if (isFirstWaypoint) {
                        // 第一个路径点：设置初始位置
                        tracker.setPose(new Pose2D(
                                DistanceUnit.INCH, x, y,
                                AngleUnit.DEGREES, heading
                        ));

                        if (task != null) {
                            Log.i("SplineAuto", marker != null ? marker : command);
                            tracker.startMove(x, y, dx, dy, task);
                        } else {
                            Log.w("SplineAuto", "[BRK1/3] waypoint[0] (FIRST): task=null, robot moves but command WONT run");
                            tracker.startMove(x, y, dx, dy);
                        }

                        // 同步朝向，覆盖 startMove 从里程计读取的朝向
                        tracker.heading = heading;
                        tracker.preH = heading;

                        isFirstWaypoint = false;
                    } else {
                        if (task != null) {
                            Log.i("SplineAuto", marker != null ? marker : command);
                            tracker.addPoint(x, y, dx, dy, heading, task);
                        } else {
                            Log.w("SplineAuto", "[BRK1/3] waypoint[" + i + "]: task=null, robot moves but command WONT run");
                            tracker.addPoint(x, y, dx, dy, heading);
                        }
                    }

//                    // delayAfterArrive：到达后等待（AzConductor 兼容）
//                    if (delayAfterArrive > 0) {
//                        tracker.addTime(delayAfterArrive);
//                    }
                }
            }

        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    /**
     * 静态便捷方法。
     */
    public static void executeJsonTrajectory(String jsonString, SplineTracker tracker) {
        new SplineTrajectoryLoader(tracker).execute(jsonString);
    }

    /**
     * 将 JSON 中的 command 名称解析为可执行的 Runnable。
     * 通过 {@link HttpJsonService#createCommandRunnable(String, String[])}
     * 桥接 AzConductor 的 command/commandParams 字段与 @AutoTask 命令注册表。
     */
    private Runnable resolveCommandTask(String commandName, String[] commandParams) {
        Runnable task = HttpJsonService.createCommandRunnable(commandName, commandParams);
        if (task != null) {
            Log.i("SplineAuto", "[BRK1-OK] Resolved command task: " + commandName + ", params=" + java.util.Arrays.toString(commandParams));
        } else {
            Log.w("SplineAuto", "[BRK1] Unresolved command: '" + commandName + "' — check /commands endpoint for matching name");
        }
        return task;
    }
}
