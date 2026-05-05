package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
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
    public static double _2a_testPeriodSec    = 2.0;   // 正弦波週期 (秒)
    public static double _2b_testAmplitudeDeg = 45.0;  // 振幅 (度)

    // ===== 私有成員 =====
    private SwerveSubsystem swerve;
    private FtcDashboard    dashboard;

    // D-on-Measurement 狀態（每輪各自追蹤）
    private double flPrevMeas, frPrevMeas, blPrevMeas, brPrevMeas;
    private double flFilteredD, frFilteredD, blFilteredD, brFilteredD;
    private long   prevTimeNs;
    private boolean pidSeeded = false;
    private static final double D_FILTER_ALPHA = 0.3;

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

        // 初始化 D-on-Measurement 狀態
        flPrevMeas = swerve.getFrontLeft() .getTurningPosition();
        frPrevMeas = swerve.getFrontRight().getTurningPosition();
        blPrevMeas = swerve.getBackLeft()  .getTurningPosition();
        brPrevMeas = swerve.getBackRight() .getTurningPosition();
        prevTimeNs = System.nanoTime();
        pidSeeded  = true;

        try {
            while (opModeIsActive()) {

                // ── 驅動馬達鎖死 ──
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

                // ── 計算輸出（D-on-Measurement，與 SwerveModule 完全一致）──
                SwerveModuleState targetState = new SwerveModuleState(1.0, new Rotation2d(targetRad));

                double[] flResult = computeTurningOutputWithError(targetState, flAngle, 0);
                double[] frResult = computeTurningOutputWithError(targetState, frAngle, 1);
                double[] blResult = computeTurningOutputWithError(targetState, blAngle, 2);
                double[] brResult = computeTurningOutputWithError(targetState, brAngle, 3);

                double flOutput = flResult[0];  double flErrDeg = flResult[1];
                double frOutput = frResult[0];  double frErrDeg = frResult[1];
                double blOutput = blResult[0];  double blErrDeg = blResult[1];
                double brOutput = brResult[0];  double brErrDeg = brResult[1];

                swerve.getFrontLeft() .setTurningPower(flOutput);
                swerve.getFrontRight().setTurningPower(frOutput);
                swerve.getBackLeft()  .setTurningPower(blOutput);
                swerve.getBackRight() .setTurningPower(brOutput);

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
                telemetry.addData("Target", "%.1f°  (±%.0f°, T=%.1fs)",
                        targetDeg, _2b_testAmplitudeDeg, _2a_testPeriodSec);
                telemetry.addLine("─────────────────────────");
                telemetry.addData("FL", "%.1f° err:%.1f° out:%.2f", Math.toDegrees(flAngle), flErrDeg, flOutput);
                telemetry.addData("FR", "%.1f° err:%.1f° out:%.2f", Math.toDegrees(frAngle), frErrDeg, frOutput);
                telemetry.addData("BL", "%.1f° err:%.1f° out:%.2f", Math.toDegrees(blAngle), blErrDeg, blOutput);
                telemetry.addData("BR", "%.1f° err:%.1f° out:%.2f", Math.toDegrees(brAngle), brErrDeg, brOutput);
                telemetry.update();

                // 更新全域時間戳（4 輪共用同一個 dt）
                prevTimeNs = System.nanoTime();
            }

        } finally {
            stopDriveMotors();
            stopTurningMotors();
        }
    }

    // ═══════════════════════════════════════════════════════
    //  私有輔助方法
    // ═══════════════════════════════════════════════════════

    /**
     * 計算單輪轉向 PD 輸出（D-on-Measurement），同時回傳誤差（供 Graph 顯示）。
     * 邏輯與 SwerveModule.computeTurningOutput() 完全一致。
     *
     * @param wheelIndex 0=FL, 1=FR, 2=BL, 3=BR（用於存取各輪的 prevMeas）
     * @return double[2]：[0]=輸出功率, [1]=optimize 後的有號誤差（度）
     */
    private double[] computeTurningOutputWithError(SwerveModuleState targetState,
                                                   double currentAngle,
                                                   int wheelIndex) {
        SwerveModuleState optimized = SwerveModuleState.optimize(
                targetState, new Rotation2d(currentAngle));

        double error    = Math.IEEEremainder(optimized.angle.getRadians() - currentAngle, 2 * Math.PI);
        double errorDeg = Math.toDegrees(error);

        // 讀取 / 更新該輪的 prevMeas
        double prevMeas = getPrevMeas(wheelIndex);

        if (Math.abs(errorDeg) <= TuningConfig.deadbandDeg()) {
            setPrevMeas(wheelIndex, currentAngle);
            setFilteredD(wheelIndex, 0);
            return new double[]{0, errorDeg};
        }

        // P 項
        double pTerm = TuningConfig.turningP() * error;

        // D 項（D-on-Measurement + 低通濾波）
        double fD = getFilteredD(wheelIndex);
        long nowNs = System.nanoTime();
        if (pidSeeded) {
            double dt = (nowNs - prevTimeNs) * 1e-9;
            if (dt > 1e-6) {
                double dMeas = Math.IEEEremainder(currentAngle - prevMeas, 2 * Math.PI);
                double rawD = -TuningConfig.turningD() * dMeas / dt;
                fD = D_FILTER_ALPHA * rawD + (1 - D_FILTER_ALPHA) * fD;
                setFilteredD(wheelIndex, fD);
            }
        }
        setPrevMeas(wheelIndex, currentAngle);

        double output = (pTerm + fD) * TuningConfig.turningOutputScale();
        output = Math.max(-1.0, Math.min(1.0, output));

        // CRServo 靜摩擦補償
        double minOut = TuningConfig.minOutput();
        if (minOut > 0 && Math.abs(errorDeg) > TuningConfig.minOutputThreshDeg() && Math.abs(output) < minOut) {
            output = Math.copySign(minOut, output);
        }

        return new double[]{output, errorDeg};
    }

    private double getPrevMeas(int idx) {
        switch (idx) {
            case 0: return flPrevMeas;
            case 1: return frPrevMeas;
            case 2: return blPrevMeas;
            default: return brPrevMeas;
        }
    }

    private void setPrevMeas(int idx, double val) {
        switch (idx) {
            case 0: flPrevMeas = val; break;
            case 1: frPrevMeas = val; break;
            case 2: blPrevMeas = val; break;
            default: brPrevMeas = val; break;
        }
    }

    private double getFilteredD(int idx) {
        switch (idx) {
            case 0: return flFilteredD;
            case 1: return frFilteredD;
            case 2: return blFilteredD;
            default: return brFilteredD;
        }
    }

    private void setFilteredD(int idx, double val) {
        switch (idx) {
            case 0: flFilteredD = val; break;
            case 1: frFilteredD = val; break;
            case 2: blFilteredD = val; break;
            default: brFilteredD = val; break;
        }
    }

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
