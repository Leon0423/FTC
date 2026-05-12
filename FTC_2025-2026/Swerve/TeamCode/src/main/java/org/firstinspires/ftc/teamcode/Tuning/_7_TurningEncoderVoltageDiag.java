package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/**
 * 診斷工具：高頻取樣 Turning Encoder，偵測電壓波動與讀數突跳的關聯性。
 *
 * 使用方式：
 *   1. 按 START 開始記錄
 *   2. 觀察 telemetry 上的即時數據與統計
 *   3. Gamepad1.A = 重置統計  Gamepad1.B = 暫停/繼續取樣
 *   4. 若 spike 數量與 busVoltage 下降同步出現，代表電壓問題
 */
@TeleOp(name = "7. Turning Encoder Voltage Diag", group = "Tuning")
public class _7_TurningEncoderVoltageDiag extends LinearOpMode {

    private static final double SPIKE_THRESHOLD_DEG = 3.0;
    private static final int HISTORY_SIZE = 200;

    private static class EncoderStats {
        final String label;
        final AnalogInput encoder;

        double prevDeg = Double.NaN;
        double minDeg = Double.MAX_VALUE;
        double maxDeg = -Double.MAX_VALUE;
        double sumDeg = 0;
        int sampleCount = 0;
        int spikeCount = 0;
        double lastSpikeDelta = 0;
        double maxSpikeDelta = 0;

        double minVoltage = Double.MAX_VALUE;
        double maxVoltage = -Double.MAX_VALUE;

        final double[] recentDeg = new double[HISTORY_SIZE];
        final double[] recentVoltage = new double[HISTORY_SIZE];
        int historyIdx = 0;

        double jitterSum = 0;

        EncoderStats(String label, AnalogInput encoder) {
            this.label = label;
            this.encoder = encoder;
        }

        void reset() {
            prevDeg = Double.NaN;
            minDeg = Double.MAX_VALUE;
            maxDeg = -Double.MAX_VALUE;
            sumDeg = 0;
            sampleCount = 0;
            spikeCount = 0;
            lastSpikeDelta = 0;
            maxSpikeDelta = 0;
            minVoltage = Double.MAX_VALUE;
            maxVoltage = -Double.MAX_VALUE;
            historyIdx = 0;
            jitterSum = 0;
        }

        void sample() {
            if (encoder == null) return;

            double v = encoder.getVoltage();
            double maxV = encoder.getMaxVoltage();
            if (maxV == 0) return;

            double deg = (v / maxV) * 360.0;

            if (v < minVoltage) minVoltage = v;
            if (v > maxVoltage) maxVoltage = v;
            if (deg < minDeg) minDeg = deg;
            if (deg > maxDeg) maxDeg = deg;

            sumDeg += deg;
            sampleCount++;

            if (!Double.isNaN(prevDeg)) {
                double delta = Math.abs(deg - prevDeg);
                if (delta > 180) delta = 360 - delta;
                jitterSum += delta;

                if (delta > SPIKE_THRESHOLD_DEG) {
                    spikeCount++;
                    lastSpikeDelta = delta;
                    if (delta > maxSpikeDelta) maxSpikeDelta = delta;
                }
            }
            prevDeg = deg;

            recentDeg[historyIdx % HISTORY_SIZE] = deg;
            recentVoltage[historyIdx % HISTORY_SIZE] = v;
            historyIdx++;
        }

        double getAvgDeg() {
            return sampleCount > 0 ? sumDeg / sampleCount : 0;
        }

        double getAvgJitter() {
            return sampleCount > 1 ? jitterSum / (sampleCount - 1) : 0;
        }

        double getRecentStdDev() {
            int count = Math.min(sampleCount, HISTORY_SIZE);
            if (count < 2) return 0;

            double sum = 0;
            for (int i = 0; i < count; i++) sum += recentDeg[i];
            double mean = sum / count;

            double sqSum = 0;
            for (int i = 0; i < count; i++) {
                double diff = recentDeg[i] - mean;
                sqSum += diff * diff;
            }
            return Math.sqrt(sqSum / (count - 1));
        }
    }

    @Override
    public void runOpMode() {
        AnalogInput flEnc = null, frEnc = null, blEnc = null, brEnc = null;
        boolean initOk = true;
        StringBuilder err = new StringBuilder();

        try { flEnc = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontLeftAbsoluteEncoderName); }
        catch (Exception e) { initOk = false; err.append("FL Encoder 找不到\n"); }

        try { frEnc = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontRightAbsoluteEncoderName); }
        catch (Exception e) { initOk = false; err.append("FR Encoder 找不到\n"); }

        try { blEnc = hardwareMap.get(AnalogInput.class, DriveConstants.kBackLeftAbsoluteEncoderName); }
        catch (Exception e) { initOk = false; err.append("BL Encoder 找不到\n"); }

        try { brEnc = hardwareMap.get(AnalogInput.class, DriveConstants.kBackRightAbsoluteEncoderName); }
        catch (Exception e) { initOk = false; err.append("BR Encoder 找不到\n"); }

        VoltageSensor batteryVoltage = hardwareMap.voltageSensor.iterator().next();

        EncoderStats fl = new EncoderStats("FL", flEnc);
        EncoderStats fr = new EncoderStats("FR", frEnc);
        EncoderStats bl = new EncoderStats("BL", blEnc);
        EncoderStats br = new EncoderStats("BR", brEnc);
        EncoderStats[] all = {fl, fr, bl, br};

        while (!isStarted() && !isStopRequested()) {
            if (!initOk) {
                telemetry.addLine("⚠ 硬體初始化失敗！");
                telemetry.addLine(err.toString());
            } else {
                telemetry.addLine("== Turning Encoder 電壓診斷 ==");
                telemetry.addLine("按 START 開始高頻取樣");
                telemetry.addLine("A = 重置統計  B = 暫停/繼續");
                telemetry.addData("Spike 閾值", "%.1f°", SPIKE_THRESHOLD_DEG);
                telemetry.addData("Battery", "%.2fV", batteryVoltage.getVoltage());
            }
            telemetry.update();
            idle();
        }

        if (!initOk) return;

        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        boolean paused = false;
        boolean prevA = false;
        boolean prevB = false;

        double busVMin = Double.MAX_VALUE;
        double busVMax = -Double.MAX_VALUE;
        int totalLoops = 0;

        while (opModeIsActive()) {
            boolean curA = gamepad1.a;
            boolean curB = gamepad1.b;

            if (curA && !prevA) {
                for (EncoderStats s : all) s.reset();
                busVMin = Double.MAX_VALUE;
                busVMax = -Double.MAX_VALUE;
                totalLoops = 0;
                runtime.reset();
            }
            if (curB && !prevB) {
                paused = !paused;
            }
            prevA = curA;
            prevB = curB;

            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            if (!paused) {
                for (EncoderStats s : all) s.sample();
                totalLoops++;
            }

            double busV = batteryVoltage.getVoltage();
            if (busV < busVMin) busVMin = busV;
            if (busV > busVMax) busVMax = busV;

            telemetry.addLine(paused ? "⏸ 已暫停" : "▶ 取樣中");
            telemetry.addData("取樣數", "%d  (%.0f Hz)", totalLoops, loopMs > 0 ? 1000.0 / loopMs : 0);
            telemetry.addData("運行時間", "%.1f s", runtime.seconds());
            telemetry.addLine("");

            telemetry.addLine("── Battery ──");
            telemetry.addData("Bus Voltage", "%.2fV  (min %.2f / max %.2f / Δ%.3fV)",
                    busV, busVMin, busVMax, busVMax - busVMin);
            telemetry.addLine("");

            for (EncoderStats s : all) {
                if (s.encoder == null) {
                    telemetry.addData(s.label, "N/A");
                    continue;
                }
                telemetry.addLine(String.format("── %s ──", s.label));
                telemetry.addData("  Raw", "%.2f°  (V: %.3fV)", s.prevDeg, s.encoder.getVoltage());
                telemetry.addData("  Range", "%.2f° ~ %.2f°  (Δ%.2f°)", s.minDeg, s.maxDeg, s.maxDeg - s.minDeg);
                telemetry.addData("  Avg", "%.2f°  StdDev: %.3f°", s.getAvgDeg(), s.getRecentStdDev());
                telemetry.addData("  Jitter", "avg %.3f°/sample", s.getAvgJitter());
                telemetry.addData("  Enc V Range", "%.3f ~ %.3fV  (Δ%.4fV)", s.minVoltage, s.maxVoltage, s.maxVoltage - s.minVoltage);
                telemetry.addData("  Spikes", "%d  (last Δ%.1f°  max Δ%.1f°)",
                        s.spikeCount, s.lastSpikeDelta, s.maxSpikeDelta);
            }

            telemetry.addLine("");
            telemetry.addLine("── 判讀 ──");
            int totalSpikes = fl.spikeCount + fr.spikeCount + bl.spikeCount + br.spikeCount;
            double busRange = busVMax - busVMin;
            if (totalSpikes == 0) {
                telemetry.addLine("✓ 無突跳偵測到，encoder 讀數穩定");
            } else if (busRange > 0.5) {
                telemetry.addLine("⚠ 偵測到 spike 且 bus voltage 波動 > 0.5V");
                telemetry.addLine("  → 電壓不穩可能是主因，檢查電池/接線");
            } else {
                telemetry.addLine("⚠ 偵測到 spike 但 bus voltage 穩定");
                telemetry.addLine("  → 可能是 encoder 接線鬆動或訊號雜訊");
            }

            telemetry.update();
        }
    }
}
