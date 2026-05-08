package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * Turning CRServo PID 調整 OpMode - 四輪同時測試
 *
 * ★ PID 參數直接讀取 TuningConfig，與 Swerve_Control 完全共用同一組 Dashboard 滑桿。
 *   → Dashboard > TuningConfig 調整 _1a_turningP / _1b_turningI / _1c_turningD /
 *                                    _1d_turningOutputScale / _2a_deadbandDeg / _2b_minOutput
 *   → 此頁只有測試波形參數 (_2a_testPeriodSec / _2b_testAmplitudeDeg) 是 Tuner 獨有。
 *
 * ★ 控制邏輯與 SwerveModule.computeTurningOutput() 完全一致：
 *   optimize → normalizeAngle → deadband → PID → clamp → minOutput
 *
 * ★ 驅動馬達完全靜止，只控制轉向 CRServo。
 *
 * 使用方法：
 * 1. 連接 FTC Dashboard (http://192.168.43.1:8080/dash)
 * 2. Config > TuningConfig 調整 PID 參數（同時影響 Swerve_Control）
 * 3. Graph 觀察：target_deg / FL_deg / FR_deg / BL_deg / BR_deg
 * 4. STOP 後馬達停止。
 */
@Config
@TeleOp(name = "4. Turning PID Tuner", group = "Tuning")
public class _4_TurningPIDTuner extends LinearOpMode {

    // ===== 此 Tuner 獨有的測試波形參數（PID 參數統一在 TuningConfig）=====
    public static double  _2a_testPeriodSec    = 4.0;   // 波形週期 (秒)
    public static double  _2b_testAmplitudeDeg = 45.0;  // 振幅 (度)
    // false = 正弦波（觀察穩態追蹤）; true = 方波（測試步階響應，更接近實際操作）
    public static boolean _2c_squareWave       = false;

    // ===== 私有成員 =====
    private SwerveSubsystem swerve;
    private FtcDashboard    dashboard;



    @Override
    public void runOpMode() throws InterruptedException {

        swerve    = new SwerveSubsystem(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // 重新初始化轉向追蹤，以絕對編碼器值為起點
        swerve.getFrontLeft() .initTurningTracking();
        swerve.getFrontRight().initTurningTracking();
        swerve.getBackLeft()  .initTurningTracking();
        swerve.getBackRight() .initTurningTracking();

        telemetry.addLine("=== Turning PID Tuner (4 輪同時, D-on-Measurement) ===");
        telemetry.addLine("PID 參數：Dashboard > TuningConfig (_1a/_1b/_1c/_1d/_2a/_2b)");
        telemetry.addLine("Graph: target_deg / FL_deg / FR_deg / BL_deg / BR_deg");
        telemetry.update();

        waitForStart();

        double startTime = getRuntime();

        try {
            while (opModeIsActive()) {

                // ── 驅動馬達鎖死 ──
                stopDriveMotors();

                // ── 目標角度（正弦波 or 方波，由 _2c_squareWave 切換）──
                double elapsed = getRuntime() - startTime;
                double phase   = 2 * Math.PI * elapsed / _2a_testPeriodSec;
                double wave    = _2c_squareWave
                        ? Math.signum(Math.sin(phase))   // 方波：±1
                        : Math.sin(phase);               // 正弦波
                double targetRad = Math.toRadians(_2b_testAmplitudeDeg) * wave;

                // ── 讀取各輪實際角度 ──
                double flAngle = swerve.getFrontLeft() .getTurningPosition();
                double frAngle = swerve.getFrontRight().getTurningPosition();
                double blAngle = swerve.getBackLeft()  .getTurningPosition();
                double brAngle = swerve.getBackRight() .getTurningPosition();

                // ── 直接呼叫 SwerveModule 的 turning PID，避免 Tuner 和 TeleOp 公式分叉 ──
                swerve.getFrontLeft() .alignTurningOnly(targetRad);
                swerve.getFrontRight().alignTurningOnly(targetRad);
                swerve.getBackLeft()  .alignTurningOnly(targetRad);
                swerve.getBackRight() .alignTurningOnly(targetRad);

                double flOutput = swerve.getFrontLeft() .getTurningOutput();
                double frOutput = swerve.getFrontRight().getTurningOutput();
                double blOutput = swerve.getBackLeft()  .getTurningOutput();
                double brOutput = swerve.getBackRight() .getTurningOutput();
                double flErrDeg = Math.toDegrees(swerve.getFrontLeft() .getTurningErrorRad());
                double frErrDeg = Math.toDegrees(swerve.getFrontRight().getTurningErrorRad());
                double blErrDeg = Math.toDegrees(swerve.getBackLeft()  .getTurningErrorRad());
                double brErrDeg = Math.toDegrees(swerve.getBackRight() .getTurningErrorRad());

                // ── Dashboard 圖表 ──
                TelemetryPacket packet = new TelemetryPacket();
                double targetDeg = Math.toDegrees(targetRad);
                packet.put("target_deg", targetDeg);
                packet.put("FL_deg",     Math.toDegrees(flAngle));
                packet.put("FR_deg",     Math.toDegrees(frAngle));
                packet.put("BL_deg",     Math.toDegrees(blAngle));
                packet.put("BR_deg",     Math.toDegrees(brAngle));
                // err_deg = optimize 後的最短路徑誤差（與 PID 實際使用的誤差相同）
                packet.put("FL_err_deg", flErrDeg);
                packet.put("FR_err_deg", frErrDeg);
                packet.put("BL_err_deg", blErrDeg);
                packet.put("BR_err_deg", brErrDeg);
                packet.put("FL_out",     flOutput);
                packet.put("FR_out",     frOutput);
                packet.put("BL_out",     blOutput);
                packet.put("BR_out",     brOutput);
                dashboard.sendTelemetryPacket(packet);

                // ── Driver Station 文字顯示 ──
                telemetry.addData("P / I / D",      "%.4f / %.4f / %.4f",
                        TuningConfig.turningP(), TuningConfig.turningI(), TuningConfig.turningD());
                telemetry.addData("Scale / DB / minOut", "%.2f / %.1f° / %.3f",
                        TuningConfig.turningOutputScale(), TuningConfig.deadbandDeg(), TuningConfig.minOutput());
                telemetry.addData("Target", "%.1f°  (±%.0f°, T=%.1fs, %s)",
                        targetDeg, _2b_testAmplitudeDeg, _2a_testPeriodSec,
                        _2c_squareWave ? "方波" : "正弦波");
                telemetry.addLine("─────────────────────────");
                telemetry.addData("FL", "%.1f° err:%.1f° out:%.2f", Math.toDegrees(flAngle), flErrDeg, flOutput);
                telemetry.addData("FR", "%.1f° err:%.1f° out:%.2f", Math.toDegrees(frAngle), frErrDeg, frOutput);
                telemetry.addData("BL", "%.1f° err:%.1f° out:%.2f", Math.toDegrees(blAngle), blErrDeg, blOutput);
                telemetry.addData("BR", "%.1f° err:%.1f° out:%.2f", Math.toDegrees(brAngle), brErrDeg, brOutput);
                telemetry.update();

                }

        } finally {
            stopDriveMotors();
            stopTurningMotors();
        }
    }

    // ═══════════════════════════════════════════════════════
    //  私有輔助方法
    // ═══════════════════════════════════════════════════════

    private void stopDriveMotors() {
        swerve.getFrontLeft() .setDriveMotorPowerDirect(0);
        swerve.getFrontRight().setDriveMotorPowerDirect(0);
        swerve.getBackLeft()  .setDriveMotorPowerDirect(0);
        swerve.getBackRight() .setDriveMotorPowerDirect(0);
    }

    private void stopTurningMotors() {
        swerve.getFrontLeft() .setTurningPower(0);
        swerve.getFrontRight().setTurningPower(0);
        swerve.getBackLeft()  .setTurningPower(0);
        swerve.getBackRight() .setTurningPower(0);
    }
}
