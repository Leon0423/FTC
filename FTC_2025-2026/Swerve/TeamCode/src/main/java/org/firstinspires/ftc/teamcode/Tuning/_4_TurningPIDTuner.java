package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * Turning CRServo PID 調整 OpMode - 四輪同時測試
 *
 * ★ 保證不改變 offset 值 ★
 *   - 整個 opmode 期間 disableSaving() 保持開啟
 *   - 測試期間轉向軌跡不會儲存，不會改變已校正的 offset
 *   - 直接停止馬達即可，不對齊也不儲存
 *
 * ★ 驅動馬達完全靜止 ★
 *   - 只呼叫 setTurningPower()，不碰 driveMotor
 *
 * 使用方法：
 * 1. 連接 FTC Dashboard (http://192.168.43.1:8080/dash)
 * 2. Config > _4_TurningPIDTuner 調整參數
 * 3. Graph 觀察：target_deg, FL_deg, FR_deg, BL_deg, BR_deg
 * 4. STOP 後馬達停止，offset 保持不變
 */
@Config
@TeleOp(name = "4. Turning PID Tuner", group = "Tuning")
public class _4_TurningPIDTuner extends LinearOpMode {

    // ===== Dashboard 可調參數 =====
    // 1. Turning PID
    public static double _1a_turningP       = Constants.ModuleConstants.kPTurning;
    public static double _1b_turningI       = Constants.ModuleConstants.kITurning;
    public static double _1c_turningD       = Constants.ModuleConstants.kDTurning;
    public static double _1d_outputScale    = Constants.ModuleConstants.kTurningOutputScale;
    public static double _1e_deadbandDeg    = Constants.ModuleConstants.kTurningDeadbandDeg;

    // 2. 測試波形
    public static double _2a_testPeriodSec    = 2.0;   // 正弦波週期 (秒)
    public static double _2b_testAmplitudeDeg = 45.0;  // 振幅 (度)

    // ===== 私有成員 =====
    private SwerveSubsystem swerve;
    private FtcDashboard    dashboard;
    private PIDController   flPID, frPID, blPID, brPID;

    @Override
    public void runOpMode() throws InterruptedException {

        swerve   = new SwerveSubsystem(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // ★ 清除所有儲存的角度記憶，強制從絕對編碼器重新初始化
        // ★ 這樣測試期間的 getTurningPosition() 累積錯誤就無法被存儲
        clearAllSavedAngles();

        // ★ 關閉儲存，此後任何路徑都不會意外寫入 angle prefs
        disableSavingAll();

        // ★ 重新初始化轉向追蹤，用絕對編碼器值
        swerve.getFrontLeft() .initTurningTracking();
        swerve.getFrontRight().initTurningTracking();
        swerve.getBackLeft()  .initTurningTracking();
        swerve.getBackRight() .initTurningTracking();

        flPID = new PIDController(_1a_turningP, _1b_turningI, _1c_turningD);
        frPID = new PIDController(_1a_turningP, _1b_turningI, _1c_turningD);
        blPID = new PIDController(_1a_turningP, _1b_turningI, _1c_turningD);
        brPID = new PIDController(_1a_turningP, _1b_turningI, _1c_turningD);

        telemetry.addLine("=== Turning PID Tuner (4 輪同時) ===");
        telemetry.addLine("Graph: target_deg / FL_deg / FR_deg / BL_deg / BR_deg");
        telemetry.addLine("STOP 後自動對齊 0° 並存檔");
        telemetry.update();

        waitForStart();

        double startTime = getRuntime();

        // ★ try-finally：不管是正常 STOP 還是例外，都一定執行結束程序
        try {
            while (opModeIsActive()) {

                // ── 即時更新 PID 參數（Dashboard 滑桿有效）──
                flPID.setPID(_1a_turningP, _1b_turningI, _1c_turningD);
                frPID.setPID(_1a_turningP, _1b_turningI, _1c_turningD);
                blPID.setPID(_1a_turningP, _1b_turningI, _1c_turningD);
                brPID.setPID(_1a_turningP, _1b_turningI, _1c_turningD);

                // ── 驅動馬達鎖死（每圈一次即可）──
                stopDriveMotors();

                // ── 正弦波目標角度 ──
                double elapsed   = getRuntime() - startTime;
                double targetRad = Math.toRadians(_2b_testAmplitudeDeg)
                        * Math.sin(2 * Math.PI * elapsed / _2a_testPeriodSec);

                // ── 讀取各輪實際角度 ──
                double flAngle = swerve.getFrontLeft() .getTurningPosition();
                double frAngle = swerve.getFrontRight().getTurningPosition();
                double blAngle = swerve.getBackLeft()  .getTurningPosition();
                double brAngle = swerve.getBackRight() .getTurningPosition();

                // ── 計算 optimized 目標並輸出功率 ──
                // 使用 optimize 確保與 Swerve_Control 實際行為一致
                // speed=1.0 只是讓 optimize 有方向可判斷，不驅動任何馬達
                SwerveModuleState targetState = new SwerveModuleState(1.0, new Rotation2d(targetRad));

                double flOutput = computeTurningOutput(flPID, targetState, flAngle);
                double frOutput = computeTurningOutput(frPID, targetState, frAngle);
                double blOutput = computeTurningOutput(blPID, targetState, blAngle);
                double brOutput = computeTurningOutput(brPID, targetState, brAngle);

                swerve.getFrontLeft() .setTurningPower(flOutput);
                swerve.getFrontRight().setTurningPower(frOutput);
                swerve.getBackLeft()  .setTurningPower(blOutput);
                swerve.getBackRight() .setTurningPower(brOutput);

                // ── Dashboard 圖表（角度用度數，易讀）──
                TelemetryPacket packet = new TelemetryPacket();
                double targetDeg = Math.toDegrees(targetRad);
                packet.put("target_deg", targetDeg);
                packet.put("FL_deg",     Math.toDegrees(flAngle));
                packet.put("FR_deg",     Math.toDegrees(frAngle));
                packet.put("BL_deg",     Math.toDegrees(blAngle));
                packet.put("BR_deg",     Math.toDegrees(brAngle));
                // 誤差與輸出供進階分析
                packet.put("FL_err_deg", targetDeg - Math.toDegrees(flAngle));
                packet.put("FR_err_deg", targetDeg - Math.toDegrees(frAngle));
                packet.put("BL_err_deg", targetDeg - Math.toDegrees(blAngle));
                packet.put("BR_err_deg", targetDeg - Math.toDegrees(brAngle));
                packet.put("FL_out",     flOutput);
                packet.put("FR_out",     frOutput);
                packet.put("BL_out",     blOutput);
                packet.put("BR_out",     brOutput);
                dashboard.sendTelemetryPacket(packet);

                // ── Driver Station 文字顯示 ──
                telemetry.addData("P / I / D",  "%.4f / %.4f / %.4f",
                        _1a_turningP, _1b_turningI, _1c_turningD);
                telemetry.addData("Scale / DB", "%.2f / %.1f°",
                        _1d_outputScale, _1e_deadbandDeg);
                telemetry.addData("Target",     "%.1f°  (±%.0f°, T=%.1fs)",
                        targetDeg, _2b_testAmplitudeDeg, _2a_testPeriodSec);
                telemetry.addLine("─────────────────────────");
                telemetry.addData("FL", "%.1f° → out: %.2f", Math.toDegrees(flAngle), flOutput);
                telemetry.addData("FR", "%.1f° → out: %.2f", Math.toDegrees(frAngle), frOutput);
                telemetry.addData("BL", "%.1f° → out: %.2f", Math.toDegrees(blAngle), blOutput);
                telemetry.addData("BR", "%.1f° → out: %.2f", Math.toDegrees(brAngle), brOutput);
                telemetry.update();
            }

        } finally {
            // ★ 無論如何都執行：停止所有馬達
            stopDriveMotors();
            stopTurningMotors();

            // ★ 保持 disableSaving() 狀態，不儲存任何測試期間的角度變化
            // ★ 直接停止，不呼叫 enableSavingAll() / swerve.stopModules()
        }
    }

    // ═══════════════════════════════════════════════════════
    //  私有輔助方法
    // ═══════════════════════════════════════════════════════

    /**
     * 計算單輪轉向 PID 輸出
     * 流程與 SwerveModule.setDesiredState() 完全相同：
     *   optimize → 計算 error → PID → clamp → deadband
     */
    private double computeTurningOutput(PIDController pid,
                                        SwerveModuleState targetState,
                                        double currentAngle) {
        SwerveModuleState optimized = SwerveModuleState.optimize(
                targetState, new Rotation2d(currentAngle));

        double error    = normalizeAngle(optimized.angle.getRadians() - currentAngle);
        double errorDeg = Math.abs(Math.toDegrees(error));

        if (errorDeg < _1e_deadbandDeg) return 0;

        double output = pid.calculate(0, error) * _1d_outputScale;
        return Math.max(-1.0, Math.min(1.0, output));
    }


    /** 鎖死所有驅動馬達 */
    private void stopDriveMotors() {
        swerve.getFrontLeft() .setDriveMotorPowerDirect(0);
        swerve.getFrontRight().setDriveMotorPowerDirect(0);
        swerve.getBackLeft()  .setDriveMotorPowerDirect(0);
        swerve.getBackRight() .setDriveMotorPowerDirect(0);
    }

    /** 停止所有轉向 CRServo */
    private void stopTurningMotors() {
        swerve.getFrontLeft() .setTurningPower(0);
        swerve.getFrontRight().setTurningPower(0);
        swerve.getBackLeft()  .setTurningPower(0);
        swerve.getBackRight() .setTurningPower(0);
    }

    private void disableSavingAll() {
        swerve.getFrontLeft() .disableSaving();
        swerve.getFrontRight().disableSaving();
        swerve.getBackLeft()  .disableSaving();
        swerve.getBackRight() .disableSaving();
    }

    /**
     * 清除所有儲存的角度記憶
     * 強制下一次 initTurningTracking() 從絕對編碼器重新初始化
     * 確保測試期間的累積誤差不會被保存
     */
    private void clearAllSavedAngles() {
        swerve.getFrontLeft() .clearSavedAngle();
        swerve.getFrontRight().clearSavedAngle();
        swerve.getBackLeft()  .clearSavedAngle();
        swerve.getBackRight() .clearSavedAngle();
    }


    /** 角度標準化到 [-π, π] */
    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}