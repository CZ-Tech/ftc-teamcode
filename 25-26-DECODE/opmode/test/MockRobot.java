//
//import java.io.BufferedReader;
//import java.io.InputStreamReader;
//import java.io.OutputStream;
//import java.net.ServerSocket;
//import java.net.Socket;
//import java.nio.charset.StandardCharsets;
//import java.nio.file.Files;
//import java.nio.file.Path;
//import java.util.Scanner;
//import java.util.concurrent.ConcurrentHashMap;
//
///**
// * 本地联调用的模拟机器类，支持多条路径独立保存。
// * 不需要真实机器人/Android 环境，在 PC 上直接运行 main()，
// * 启动 HTTP 服务器（端口 8888）。
// *
// * <p>API 端点：</p>
// * <ul>
// *   <li>OPTIONS 任意路径 — CORS preflight，返回 204</li>
// *   <li>POST / — 接收 JSON 存入当前内存</li>
// *   <li>GET  / — 返回当前内存中的 JSON</li>
// *   <li>POST /save/{pathName} — 将当前 JSON 持久化到指定路径</li>
// *   <li>GET  /{pathName} — 获取指定路径已保存的 JSON（不改变当前）</li>
// *   <li>POST /load/{pathName} — 从指定路径加载 JSON 到当前内存</li>
// *   <li>POST /clear/{pathName} — 删除指定路径的持久化数据</li>
// *   <li>GET  /list — 列出所有已保存的路径名</li>
// * </ul>
// *
// * <p>CORS：所有响应均包含跨域头，允许浏览器端从任意源调用。</p>
// *
// * <pre>{@code
// *   # 发送 JSON
// *   curl -X POST http://localhost:8888/ -d '{"cmd":"move","speed":0.5}'
// *
// *   # 查看当前 JSON
// *   curl http://localhost:8888/
// *
// *   # 保存到指定路径
// *   curl -X POST http://localhost:8888/save/route1
// *
// *   # 查看指定路径的 JSON
// *   curl http://localhost:8888/route1
// *
// *   # 从指定路径加载到当前
// *   curl -X POST http://localhost:8888/load/route1
// *
// *   # 删除指定路径
// *   curl -X POST http://localhost:8888/clear/route1
// *
// *   # 列出所有路径
// *   curl http://localhost:8888/list
// * }</pre>
// */
//public class MockRobot {
//
//    private static final int PORT = 8888;
//    private static final Path DATA_DIR = Path.of("mock_robot_data");
//
//    /** 当前内存中的 JSON，volatile 保证多线程可见性 */
//    private static volatile String currentJson = "{}";
//    /** 已保存路径的内存缓存：路径名 → JSON */
//    private static final ConcurrentHashMap<String, String> savedPaths = new ConcurrentHashMap<>();
//
//    public static void main(String[] args) {
//        System.out.println("==============================================");
//        System.out.println("  MockRobot 本地联调模拟机器");
//        System.out.println("  监听端口: " + PORT);
//        System.out.println("  CORS: 已启用 (允许所有源)");
//        System.out.println("==============================================");
//        System.out.println("  API 端点:");
//        System.out.println("    POST /                  — 接收 JSON 存入内存");
//        System.out.println("    GET  /                  — 返回当前 JSON");
//        System.out.println("    POST /save/{name}       — 持久化到指定路径");
//        System.out.println("    GET  /{name}            — 查看指定路径的 JSON");
//        System.out.println("    POST /load/{name}       — 加载到当前内存");
//        System.out.println("    POST /clear/{name}      — 删除指定路径");
//        System.out.println("    GET  /list              — 列出所有路径");
//        System.out.println("==============================================");
//        System.out.println();
//        System.out.println("  键入 'q' 并回车可停止服务。");
//        System.out.println("  键入 'p' 查看当前 JSON。");
//        System.out.println();
//
//        loadAllFromDisk();
//
//        Thread serverThread = new Thread(MockRobot::runServer, "mock-http-server");
//        serverThread.setDaemon(true);
//        serverThread.start();
//
//        try (Scanner scanner = new Scanner(System.in)) {
//            while (true) {
//                String line = scanner.nextLine().trim();
//                if ("q".equalsIgnoreCase(line)) {
//                    System.out.println("MockRobot 已停止。");
//                    break;
//                }
//                if ("p".equalsIgnoreCase(line)) {
//                    System.out.println("当前 JSON: " + currentJson);
//                } else {
//                    System.out.println("未知命令。'q' 退出, 'p' 查看当前 JSON。");
//                }
//            }
//        }
//        System.exit(0);
//    }
//
//    private static void runServer() {
//        try (ServerSocket serverSocket = new ServerSocket(PORT)) {
//            System.out.println("[MockRobot] HTTP 服务已启动，等待连接...");
//            while (!Thread.currentThread().isInterrupted()) {
//                try (Socket socket = serverSocket.accept()) {
//                    socket.setSoTimeout(5000);
//                    BufferedReader reader = new BufferedReader(
//                            new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8));
//                    OutputStream output = socket.getOutputStream();
//
//                    String requestLine = reader.readLine();
//                    if (requestLine == null || requestLine.isEmpty()) {
//                        continue;
//                    }
//
//                    String method = requestLine.split(" ")[0];
//                    boolean isGet     = "GET".equalsIgnoreCase(method);
//                    boolean isPost    = "POST".equalsIgnoreCase(method);
//                    boolean isOptions = "OPTIONS".equalsIgnoreCase(method);
//                    String path = parseRequestPath(requestLine);
//
//                    System.out.println("[MockRobot] " + method + " " + path);
//
//                    int contentLength = 0;
//                    String headerLine;
//                    while ((headerLine = reader.readLine()) != null && !headerLine.isEmpty()) {
//                        if (headerLine.toLowerCase().startsWith("content-length:")) {
//                            contentLength = Integer.parseInt(headerLine.substring(15).trim());
//                        }
//                    }
//
//                    // --- OPTIONS preflight — CORS ---
//                    if (isOptions) {
//                        sendResponse(output, 204, null, null);
//                        continue;
//                    }
//
//                    // --- GET / — 返回当前 JSON ---
//                    if (isGet && "/".equals(path)) {
//                        sendResponse(output, 200, "application/json", currentJson);
//                        continue;
//                    }
//
//                    // --- POST / 带 body — 接收 JSON ---
//                    if (isPost && "/".equals(path) && contentLength > 0) {
//                        String body = readBody(reader, contentLength);
//                        currentJson = body;
//                        System.out.println("[MockRobot] 收到 JSON: " + currentJson);
//                        sendResponse(output, 200, "application/json", "{\"status\":\"success\"}");
//                        continue;
//                    }
//
//                    // --- 解析 /action/pathName 模式 ---
//                    String[] pathParts = path.split("/", 3);
//                    String action   = pathParts.length >= 2 ? pathParts[1] : "";
//                    String pathName = pathParts.length >= 3 ? pathParts[2] : "";
//
//                    // --- POST /save/{pathName} ---
//                    if (isPost && "save".equals(action) && !pathName.isEmpty()) {
//                        savedPaths.put(pathName, currentJson);
//                        saveToDisk(pathName, currentJson);
//                        System.out.println("[MockRobot] 已保存路径 '" + pathName + "': " + currentJson);
//                        sendResponse(output, 200, "application/json",
//                                "{\"status\":\"saved\",\"path\":\"" + pathName + "\"}");
//                        continue;
//                    }
//
//                    // --- GET /{pathName} — 获取指定路径的 JSON ---
//                    if (isGet && action.isEmpty() && !pathName.isEmpty()) {
//                        String saved = savedPaths.get(pathName);
//                        if (saved != null) {
//                            sendResponse(output, 200, "application/json", saved);
//                        } else {
//                            sendResponse(output, 404, "application/json",
//                                    "{\"status\":\"not_found\",\"path\":\"" + pathName + "\"}");
//                        }
//                        continue;
//                    }
//
//                    // --- POST /load/{pathName} ---
//                    if (isPost && "load".equals(action) && !pathName.isEmpty()) {
//                        String saved = savedPaths.get(pathName);
//                        if (saved != null) {
//                            currentJson = saved;
//                            System.out.println("[MockRobot] 从路径 '" + pathName + "' 加载: " + currentJson);
//                            sendResponse(output, 200, "application/json",
//                                    "{\"status\":\"loaded\",\"path\":\"" + pathName + "\",\"data\":" + currentJson + "}");
//                        } else {
//                            sendResponse(output, 404, "application/json",
//                                    "{\"status\":\"not_found\",\"path\":\"" + pathName + "\"}");
//                        }
//                        continue;
//                    }
//
//                    // --- POST /clear/{pathName} ---
//                    if (isPost && "clear".equals(action) && !pathName.isEmpty()) {
//                        savedPaths.remove(pathName);
//                        deleteFromDisk(pathName);
//                        System.out.println("[MockRobot] 已删除路径 '" + pathName + "'");
//                        sendResponse(output, 200, "application/json",
//                                "{\"status\":\"cleared\",\"path\":\"" + pathName + "\"}");
//                        continue;
//                    }
//
//                    // --- GET /list — 列出所有路径 ---
//                    if (isGet && "list".equals(action)) {
//                        String listJson = savedPaths.keySet().toString();
//                        sendResponse(output, 200, "application/json",
//                                "{\"status\":\"ok\",\"paths\":" + listJson + "}");
//                        continue;
//                    }
//
//                    // --- 不支持的请求 ---
//                    sendResponse(output, 405, "text/plain", "Method Not Allowed");
//
//                } catch (Exception e) {
//                    System.err.println("[MockRobot] 连接处理异常: " + e.getMessage());
//                }
//            }
//        } catch (Exception e) {
//            System.err.println("[MockRobot] 服务器异常: " + e.getMessage());
//        }
//    }
//
//    /** 读取请求 body */
//    private static String readBody(BufferedReader reader, int contentLength) throws Exception {
//        char[] buffer = new char[contentLength];
//        int totalRead = 0;
//        while (totalRead < contentLength) {
//            int read = reader.read(buffer, totalRead, contentLength - totalRead);
//            if (read == -1) break;
//            totalRead += read;
//        }
//        return new String(buffer);
//    }
//
//    /** 从请求行解析路径 */
//    private static String parseRequestPath(String requestLine) {
//        String[] parts = requestLine.split(" ");
//        if (parts.length >= 2) {
//            return parts[1];
//        }
//        return "/";
//    }
//
//    /** 将指定路径的数据持久化到磁盘 */
//    private static void saveToDisk(String pathName, String json) {
//        try {
//            Files.createDirectories(DATA_DIR);
//            Files.writeString(DATA_DIR.resolve(pathName + ".json"), json, StandardCharsets.UTF_8);
//        } catch (Exception e) {
//            System.err.println("[MockRobot] 磁盘保存失败 '" + pathName + "': " + e.getMessage());
//        }
//    }
//
//    /** 从磁盘加载所有已保存的路径到内存 */
//    private static void loadAllFromDisk() {
//        try {
//            if (!Files.isDirectory(DATA_DIR)) {
//                System.out.println("[MockRobot] 无本地数据目录，跳过加载。");
//                return;
//            }
//            try (java.util.stream.Stream<Path> files = Files.list(DATA_DIR)) {
//                files.filter(f -> f.toString().endsWith(".json")).forEach(f -> {
//                    try {
//                        String name = f.getFileName().toString();
//                        name = name.substring(0, name.length() - 5);
//                        String json = Files.readString(f, StandardCharsets.UTF_8);
//                        savedPaths.put(name, json);
//                        System.out.println("[MockRobot] 从磁盘加载路径 '" + name + "': " + json);
//                    } catch (Exception e) {
//                        System.err.println("[MockRobot] 加载文件失败: " + e.getMessage());
//                    }
//                });
//            }
//        } catch (Exception e) {
//            System.err.println("[MockRobot] 磁盘加载失败: " + e.getMessage());
//        }
//    }
//
//    /** 从磁盘删除指定路径 */
//    private static void deleteFromDisk(String pathName) {
//        try {
//            Files.deleteIfExists(DATA_DIR.resolve(pathName + ".json"));
//        } catch (Exception e) {
//            System.err.println("[MockRobot] 磁盘删除失败 '" + pathName + "': " + e.getMessage());
//        }
//    }
//
//    /**
//     * 发送 HTTP 响应（含 CORS 头）。
//     * @param contentType 可为 null（如 204 No Content）
//     * @param body 可为 null
//     */
//    private static void sendResponse(OutputStream out, int statusCode,
//                                      String contentType, String body) throws Exception {
//        byte[] bodyBytes = (body != null) ? body.getBytes(StandardCharsets.UTF_8) : null;
//        String statusText = statusCode == 200 ? "OK"
//                : statusCode == 204 ? "No Content"
//                : statusCode == 404 ? "Not Found" : "ERROR";
//        StringBuilder header = new StringBuilder();
//        header.append("HTTP/1.1 ").append(statusCode).append(" ").append(statusText).append("\r\n");
//        if (contentType != null) {
//            header.append("Content-Type: ").append(contentType).append("\r\n");
//        }
//        header.append("Access-Control-Allow-Origin: *\r\n");
//        header.append("Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n");
//        header.append("Access-Control-Allow-Headers: Content-Type\r\n");
//        header.append("Access-Control-Max-Age: 86400\r\n");
//        if (bodyBytes != null) {
//            header.append("Content-Length: ").append(bodyBytes.length).append("\r\n");
//        } else {
//            header.append("Content-Length: 0\r\n");
//        }
//        header.append("Connection: close\r\n");
//        header.append("\r\n");
//        out.write(header.toString().getBytes(StandardCharsets.UTF_8));
//        if (bodyBytes != null) {
//            out.write(bodyBytes);
//        }
//        out.flush();
//    }
//
//    public static String getCurrentJson() {
//        return currentJson;
//    }
//
//    public static String getSavedJson(String pathName) {
//        return savedPaths.get(pathName);
//    }
//}
