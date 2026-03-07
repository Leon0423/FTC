package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * Drive PID 調整專用 OpMode
 *
 * 使用方法：
 * 1. 連接 FTC Dashboard (http://192.168.43.1:8080/dash)
 * 2. 在 Config 頁籤調整 TuningConfig 中的 driveP, driveI, driveD, driveF
 * 3. 在 Graph 頁籤觀察以下數值：
 *    - targetVel: 目標速度 (m/s)
 *    - currentVel: 當前速度 (m/s)
 *    - driveError: 速度誤差 (m/s)
 *    - driveOutput: PID輸出 (-1 到 1)
 *
 * PID + F 調整建議：
 * - F (前饋): 基於目標速度的基本功率，建議先調這個
 *   設定方式：讓馬達達到目標速度所需的大約功率比例
 *   例如：如果目標 1 m/s 需要約 50% 功率，則 F ≈ 0.5
 * - P (比例): 控制速度響應，修正剩餘誤差
 * - I (積分): 消除穩態誤差，通常設為 0 或很小的值
 * - D (微分): 抑制震盪，速度控制通常不需要
 *
 * 調整順序：
 * 1. 先將 P, I, D 設為 0
 * 2. 調整 F 使馬達在穩定目標速度下能大致達到（可能有小誤差）
 * 3. 增加 P 來修正剩餘誤差
 * 4. 如果有持續的穩態誤差，小心增加 I
 * 5. 如果有震盪，嘗試增加 D 或降低 P
 */
@Config
@TeleOp(name = "5. Drive PID Tuner", group = "Tuning")
public class _5_DrivePIDTuner extends LinearOpMode {

    // 可在 Dashboard 調整的測試目標速度 (m/s)
    public static double testTargetVelocity = 0.5;

    // 選擇要測試的模組 (0=FL, 1=FR, 2=BL, 3=BR)
    public static int moduleIndex = 0;

    // 自動測試模式：週期性改變目標速度
    public static boolean autoTestMode = true;
    public static double autoTestPeriodSeconds = 4.0;
    public static double autoTestMaxVelocity = 1.0;  // m/s

    private SwerveSubsystem swerveSubsystem;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        swerveSubsystem = new SwerveSubsystem(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        // 合併 telemetry 以便同時輸出到 Driver Station 和 Dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("=== Drive PID Tuner ===");
        telemetry.addLine("在 Dashboard 的 Config 調整:");
        telemetry.addLine("  TuningConfig.driveP/I/D/F");
        telemetry.addLine("");
        telemetry.addLine("在 Graph 頁籤觀察:");
        telemetry.addLine("  targetVel, currentVel, driveError, driveOutput");
        telemetry.addLine("");
        telemetry.addLine("控制按鍵:");
        telemetry.addLine("  X = 暫停測試，啟動手動控制");
        telemetry.addLine("  Y = 繼續自動測試");
        telemetry.addLine("");
        telemetry.addLine("注意：Drive PID 需要在地板上測試");
        telemetry.addLine("      請確保有足夠空間讓機器人移動！");
        telemetry.addLine("");
        telemetry.addLine("按 Start 開始");
        telemetry.update();

        waitForStart();

        double startTime = getRuntime();
        boolean isPaused = false;  // 暫停模式標記
        boolean lastX = false;     // 用於邊緣檢測
        boolean lastY = false;

        while (opModeIsActive()) {
            // 按鍵邊緣檢測
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;

            // X 按下：暫停測試，進入手動控制
            if (currentX && !lastX) {
                isPaused = true;
                swerveSubsystem.stopModules();
            }
            // Y 按下：繼續測試
            if (currentY && !lastY) {
                isPaused = false;
                startTime = getRuntime();  // 重置計時器
            }

            lastX = currentX;
            lastY = currentY;

            // 暫停模式：手動底盤控制（無 PID）
            if (isPaused) {
                runManualControl();
                continue;
            }

            // 計算目標速度
            double targetVelocity;
            if (autoTestMode) {
                // 自動測試模式：使用正弦波產生週期性目標（只有正向）
                double elapsedTime = getRuntime() - startTime;
                double phase = (elapsedTime / autoTestPeriodSeconds) * 2 * Math.PI;
                // 使用 (sin + 1) / 2 使速度在 0 和 max 之間變化
                targetVelocity = autoTestMaxVelocity * (Math.sin(phase) + 1) / 2;
            } else {
                // 手動模式：使用 Dashboard 設定的目標
                targetVelocity = testTargetVelocity;
            }

            // 選擇要測試的模組
            SwerveModule testModule;
            String moduleName;
            switch (moduleIndex) {
                case 1:
                    testModule = swerveSubsystem.getFrontRight();
                    moduleName = "FR";
                    break;
                case 2:
                    testModule = swerveSubsystem.getBackLeft();
                    moduleName = "BL";
                    break;
                case 3:
                    testModule = swerveSubsystem.getBackRight();
                    moduleName = "BR";
                    break;
                default:
                    testModule = swerveSubsystem.getFrontLeft();
                    moduleName = "FL";
                    break;
            }

            // 設定模組狀態（角度設為 0，只測試驅動）
            SwerveModuleState state = new SwerveModuleState(targetVelocity, new Rotation2d(0));
            testModule.setDesiredState(state);

            // 停止其他模組
            stopOtherModules(moduleIndex);

            // 發送數據到 Dashboard 圖表
            TelemetryPacket packet = new TelemetryPacket();

            // 主要 Drive PID 監控數據 (用於圖表)
            packet.put("targetVel", testModule.getTargetVelocity());
            packet.put("currentVel", testModule.getCurrentVelocityMps());
            packet.put("driveError", testModule.getDriveError());
            packet.put("driveOutput", testModule.getDriveOutput());

            // 同時顯示 Turning 數據供參考
            packet.put("turnTarget", testModule.getTargetAngleRad());
            packet.put("turnCurrent", testModule.getCurrentAngleRad());

            dashboard.sendTelemetryPacket(packet);

            // 文字 telemetry
            telemetry.addData("模組", moduleName);
            telemetry.addData("模式", autoTestMode ? "自動測試" : "手動");
            telemetry.addData("Drive PID", TuningConfig.enableDrivePID() ? "啟用" : "關閉");
            telemetry.addLine("");
            telemetry.addData("=== Drive PID 參數 ===", "");
            telemetry.addData("P", "%.4f", TuningConfig.driveP());
            telemetry.addData("I", "%.4f", TuningConfig.driveI());
            telemetry.addData("D", "%.4f", TuningConfig.driveD());
            telemetry.addData("F (前饋)", "%.4f", TuningConfig.driveF());
            telemetry.addData("Output Scale", "%.2f", TuningConfig.driveOutputScale());
            telemetry.addLine("");
            telemetry.addData("=== 當前數據 ===", "");
            telemetry.addData("目標速度", "%.3f m/s", testModule.getTargetVelocity());
            telemetry.addData("當前速度", "%.3f m/s", testModule.getCurrentVelocityMps());
            telemetry.addData("速度誤差", "%.3f m/s", testModule.getDriveError());
            telemetry.addData("輸出功率", "%.3f", testModule.getDriveOutput());
            telemetry.update();
        }

        // 停止所有模組
        swerveSubsystem.stopModules();
    }

    /**
     * 停止除了測試模組以外的其他模組
     */
    private void stopOtherModules(int activeIndex) {
        if (activeIndex != 0) swerveSubsystem.getFrontLeft().stop();
        if (activeIndex != 1) swerveSubsystem.getFrontRight().stop();
        if (activeIndex != 2) swerveSubsystem.getBackLeft().stop();
        if (activeIndex != 3) swerveSubsystem.getBackRight().stop();
    }

    /**
     * 手動 Swerve 底盤控制模式（無 Drive PID，直接功率輸出）
     * 用於暫停測試時手動移動機器人
     */
    private void runManualControl() {
        // 讀取搖桿輸入
        double forward = -gamepad1.left_stick_y;  // 前進/後退
        double strafe = gamepad1.left_stick_x;    // 左右平移
        double rotate = gamepad1.right_stick_x;   // 旋轉

        // 簡單的死區處理
        double deadzone = 0.1;
        if (Math.abs(forward) < deadzone) forward = 0;
        if (Math.abs(strafe) < deadzone) strafe = 0;
        if (Math.abs(rotate) < deadzone) rotate = 0;

        // 降低速度以便精確控制
        double speedMultiplier = 0.5;
        forward *= speedMultiplier;
        strafe *= speedMultiplier;
        rotate *= speedMultiplier;

        // 使用 SwerveSubsystem 的無 PID 驅動方法
        swerveSubsystem.driveNoPID(forward, strafe, rotate);

        // 顯示手動控制狀態
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("mode", "MANUAL");
        packet.put("targetVel", 0);
        packet.put("currentVel", 0);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("=== 手動 Swerve 控制模式 ===");
        telemetry.addLine("（測試暫停中，Drive PID 關閉）");
        telemetry.addLine("");
        telemetry.addLine("左搖桿: 平移移動");
        telemetry.addLine("右搖桿: 旋轉");
        telemetry.addLine("");
        telemetry.addLine("按 Y 繼續自動測試");
        telemetry.addLine("");
        telemetry.addData("前進/後退", "%.2f", forward);
        telemetry.addData("左右平移", "%.2f", strafe);
        telemetry.addData("旋轉", "%.2f", rotate);
        telemetry.update();
    }
}

