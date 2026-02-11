package org.firstinspires.ftc.teamcode;

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
 * PID 調整專用 OpMode
 *
 * 使用方法：
 * 1. 連接 FTC Dashboard (http://192.168.43.1:8080/dash)
 * 2. 在 Config 頁籤調整 TuningConfig 中的 turningP, turningI, turningD
 * 3. 在 Graph 頁籤觀察以下數值：
 *    - FL_target: 目標角度 (弧度)
 *    - FL_current: 當前角度 (弧度)
 *    - FL_error: 誤差 (弧度)
 *    - FL_output: PID輸出 (-1 到 1)
 *
 * PID 調整建議：
 * - P (比例): 控制反應速度，太大會震盪，太小反應慢
 * - I (積分): 消除穩態誤差，通常設為 0 或很小的值
 * - D (微分): 抑制震盪，太大會導致反應變慢
 *
 * 調整順序：
 * 1. 先將 I 和 D 設為 0
 * 2. 慢慢增加 P 直到系統開始震盪，然後減少約 20-30%
 * 3. 如果有穩態誤差，慢慢增加 I
 * 4. 如果有震盪，增加 D
 */
@Config
@TeleOp(name = "Turning PID Tuner", group = "Tuning")
public class TurningPIDTuner extends LinearOpMode {

    // 可在 Dashboard 調整的測試目標角度 (度數)
    public static double testTargetDegrees = 0;

    // 選擇要測試的模組 (0=FL, 1=FR, 2=BL, 3=BR)
    public static int moduleIndex = 0;

    // 自動測試模式：週期性改變目標角度
    public static boolean autoTestMode = true;
    public static double autoTestPeriodSeconds = 3.0;
    public static double autoTestAmplitudeDegrees = 90;

    private SwerveSubsystem swerveSubsystem;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        swerveSubsystem = new SwerveSubsystem(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        // 合併 telemetry 以便同時輸出到 Driver Station 和 Dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("=== Turning PID Tuner ===");
        telemetry.addLine("在 Dashboard 的 Config 調整:");
        telemetry.addLine("  TuningConfig.turningP/I/D");
        telemetry.addLine("");
        telemetry.addLine("在 Graph 頁籤觀察:");
        telemetry.addLine("  target, current, error, output");
        telemetry.addLine("");
        telemetry.addLine("按 Start 開始");
        telemetry.update();

        waitForStart();

        double startTime = getRuntime();

        while (opModeIsActive()) {
            // 計算目標角度
            double targetDegrees;
            if (autoTestMode) {
                // 自動測試模式：使用正弦波產生週期性目標
                double elapsedTime = getRuntime() - startTime;
                double phase = (elapsedTime / autoTestPeriodSeconds) * 2 * Math.PI;
                targetDegrees = autoTestAmplitudeDegrees * Math.sin(phase);
            } else {
                // 手動模式：使用 Dashboard 設定的目標
                targetDegrees = testTargetDegrees;
            }

            double targetRadians = Math.toRadians(targetDegrees);

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

            // 設定模組狀態 (速度為 0，只測試轉向)
            SwerveModuleState state = new SwerveModuleState(0.001, new Rotation2d(targetRadians));
            testModule.setDesiredState(state);

            // 停止其他模組
            stopOtherModules(moduleIndex);

            // 發送數據到 Dashboard 圖表
            TelemetryPacket packet = new TelemetryPacket();

            // 主要 PID 監控數據 (用於圖表)
            packet.put("target", testModule.getTargetAngleRad());
            packet.put("current", testModule.getCurrentAngleRad());
            packet.put("error", testModule.getTurningErrorRad());
            packet.put("output", testModule.getTurningOutput());

            // 轉換為度數便於理解
            packet.put("target_deg", Math.toDegrees(testModule.getTargetAngleRad()));
            packet.put("current_deg", Math.toDegrees(testModule.getCurrentAngleRad()));
            packet.put("error_deg", Math.toDegrees(testModule.getTurningErrorRad()));

            dashboard.sendTelemetryPacket(packet);

            // 文字 telemetry
            telemetry.addData("模組", moduleName);
            telemetry.addData("模式", autoTestMode ? "自動測試" : "手動");
            telemetry.addLine("");
            telemetry.addData("=== PID 參數 ===", "");
            telemetry.addData("P", "%.4f", TuningConfig.turningP);
            telemetry.addData("I", "%.4f", TuningConfig.turningI);
            telemetry.addData("D", "%.4f", TuningConfig.turningD);
            telemetry.addData("Output Scale", "%.2f", TuningConfig.turningOutputScale);
            telemetry.addLine("");
            telemetry.addData("=== 當前數據 ===", "");
            telemetry.addData("目標角度", "%.1f°", Math.toDegrees(testModule.getTargetAngleRad()));
            telemetry.addData("當前角度", "%.1f°", Math.toDegrees(testModule.getCurrentAngleRad()));
            telemetry.addData("誤差", "%.2f° (%.3f rad)",
                    Math.toDegrees(testModule.getTurningErrorRad()),
                    testModule.getTurningErrorRad());
            telemetry.addData("輸出", "%.3f", testModule.getTurningOutput());
            telemetry.update();

            sleep(20); // 約 50Hz 更新率
        }

        swerveSubsystem.stopModules();
    }

    private void stopOtherModules(int activeModuleIndex) {
        if (activeModuleIndex != 0) swerveSubsystem.getFrontLeft().stop();
        if (activeModuleIndex != 1) swerveSubsystem.getFrontRight().stop();
        if (activeModuleIndex != 2) swerveSubsystem.getBackLeft().stop();
        if (activeModuleIndex != 3) swerveSubsystem.getBackRight().stop();
    }
}

