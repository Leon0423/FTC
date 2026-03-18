package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * ════════════════════════════════════════
 *  5. Drive PID Tuner
 * ════════════════════════════════════════
 *
 * 目的：調整驅動馬達 PID，讓速度精確跟隨目標
 *
 * 使用步驟：
 *   Init  │ 選擇測試模組（A=全部 / DPAD 選單顆）
 *   Start │ 自動週期性切換速度（0 → Max → 0）
 *          │ 在 Dashboard Graph 觀察 targetVel vs currentVel
 *   調整順序：
 *     1. P=0 I=0 D=0，先調 F 讓速度大致跟上
 *     2. 加 P 修正誤差
 *     3. 有持續偏差才加 I（通常不需要）
 *     4. 有震盪加 D 或降 P
 *
 * 按鍵：
 *   A        = 測試全部四顆
 *   DPAD ↑   = 只測 FL
 *   DPAD →   = 只測 FR
 *   DPAD ←   = 只測 BL
 *   DPAD ↓   = 只測 BR
 *   LB / RB  = 調整目標速度
 *   X        = 暫停 / 繼續
 */
@Config
@TeleOp(name = "5. Drive PID Tuner", group = "Tuning")
public class _5_DrivePIDTuner extends LinearOpMode {

    public static double autoTestPeriodSeconds = 2.0;  // 每段持續秒數
    public static double autoTestMaxVelocity   = 0.3;  // m/s

    private SwerveSubsystem swerve;
    private FtcDashboard dashboard;

    // 測試模式：0=全部, 1=FL, 2=FR, 3=BL, 4=BR
    private int testMode = 0;

    private boolean isPaused = false;
    private boolean lastX = false;
    private boolean lastLb = false, lastRb = false;

    @Override
    public void runOpMode() {
        swerve = new SwerveSubsystem(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        swerve.getFrontLeft().disableSaving();
        swerve.getFrontRight().disableSaving();
        swerve.getBackLeft().disableSaving();
        swerve.getBackRight().disableSaving();

        // ══════════════════════════════
        //  Init 階段：對齊輪子 + 選模組
        // ══════════════════════════════
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a)          testMode = 0;
            if (gamepad1.dpad_up)    testMode = 1;
            if (gamepad1.dpad_right) testMode = 2;
            if (gamepad1.dpad_left)  testMode = 3;
            if (gamepad1.dpad_down)  testMode = 4;

            // 對齊輪子到 0°
            swerve.getFrontLeft().alignTurningOnly(0);
            swerve.getFrontRight().alignTurningOnly(0);
            swerve.getBackLeft().alignTurningOnly(0);
            swerve.getBackRight().alignTurningOnly(0);

            telemetry.addLine("════ Drive PID Tuner ════");
            telemetry.addLine("");
            telemetry.addLine("選擇測試模組：");
            telemetry.addLine("  A = 全部四顆");
            telemetry.addLine("  DPAD ↑=FL  →=FR  ←=BL  ↓=BR");
            telemetry.addLine("");
            telemetry.addData("目前選擇", getModeLabel());
            telemetry.addData("目標速度", "%.2f m/s  (Start後 LB/RB 調整)", autoTestMaxVelocity);
            telemetry.addLine("");
            telemetry.addLine("確認後按 Start");
            telemetry.update();
        }

        if (!opModeIsActive()) return;

        double startTime = getRuntime();

        // ══════════════════════════════
        //  主測試迴圈
        // ══════════════════════════════
        while (opModeIsActive()) {

            // X 鍵暫停/繼續
            boolean x = gamepad1.x;
            if (x && !lastX) {
                isPaused = !isPaused;
                if (!isPaused) startTime = getRuntime();
            }
            lastX = x;

            // LB/RB 調整目標速度
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            if (rb && !lastRb) autoTestMaxVelocity = Math.min(Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond, autoTestMaxVelocity + 0.05);
            if (lb && !lastLb) autoTestMaxVelocity = Math.max(0.05, autoTestMaxVelocity - 0.05);
            lastRb = rb; lastLb = lb;

            if (isPaused) {
                swerve.stopModules();
                telemetry.addLine("⏸ 暫停中，按 X 繼續");
                telemetry.update();
                continue;
            }

            // 方波：每 autoTestPeriodSeconds 切換一次
            double elapsed = getRuntime() - startTime;
            int phase = (int)(elapsed / autoTestPeriodSeconds);
            double targetVel = (phase % 2 == 0) ? autoTestMaxVelocity : 0;

            // 依模式設定目標
            SwerveModuleState state = new SwerveModuleState(targetVel, new Rotation2d(0));
            applyToModules(state);

            // 取主要監測模組（全部模式取 FL）
            SwerveModule monitor = getMonitorModule();

            // Dashboard graph
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("targetVel",   monitor.getTargetVelocity());
            packet.put("currentVel",  monitor.getCurrentVelocityMps());
            packet.put("driveError",  monitor.getDriveError());
            packet.put("driveOutput", monitor.getDriveOutput());
            // 四顆分開顯示
            packet.put("FL_vel", swerve.getFrontLeft().getCurrentVelocityMps());
            packet.put("FR_vel", swerve.getFrontRight().getCurrentVelocityMps());
            packet.put("BL_vel", swerve.getBackLeft().getCurrentVelocityMps());
            packet.put("BR_vel", swerve.getBackRight().getCurrentVelocityMps());
            dashboard.sendTelemetryPacket(packet);

            // Telemetry
            telemetry.addLine("════ Drive PID Tuner ════");
            telemetry.addData("測試模組", getModeLabel());
            telemetry.addData("目標速度", "%.3f m/s  (%.0f%%)",
                    targetVel, (targetVel / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond) * 100);
            telemetry.addData("LB/RB 調整最大速度", "%.2f m/s", autoTestMaxVelocity);
            telemetry.addLine("");
            telemetry.addLine("── PID 參數 ──");
            telemetry.addData("P", "%.4f", TuningConfig.driveP());
            telemetry.addData("I", "%.4f", TuningConfig.driveI());
            telemetry.addData("D", "%.4f", TuningConfig.driveD());
            telemetry.addData("F", "%.4f", TuningConfig.driveF());
            telemetry.addLine("");
            telemetry.addLine("── 即時數據 ──");
            telemetry.addData("FL", "%.3f / %.3f m/s",
                    swerve.getFrontLeft().getTargetVelocity(),
                    swerve.getFrontLeft().getCurrentVelocityMps());
            telemetry.addData("FR", "%.3f / %.3f m/s",
                    swerve.getFrontRight().getTargetVelocity(),
                    swerve.getFrontRight().getCurrentVelocityMps());
            telemetry.addData("BL", "%.3f / %.3f m/s",
                    swerve.getBackLeft().getTargetVelocity(),
                    swerve.getBackLeft().getCurrentVelocityMps());
            telemetry.addData("BR", "%.3f / %.3f m/s",
                    swerve.getBackRight().getTargetVelocity(),
                    swerve.getBackRight().getCurrentVelocityMps());
            telemetry.addLine("");
            telemetry.addLine("X=暫停  LB/RB=調速度");
            telemetry.update();
        }

        // 結束：對齊回 0° 再存
        swerve.getFrontLeft().setDriveMotorPowerDirect(0);
        swerve.getFrontRight().setDriveMotorPowerDirect(0);
        swerve.getBackLeft().setDriveMotorPowerDirect(0);
        swerve.getBackRight().setDriveMotorPowerDirect(0);

        long alignStart = System.currentTimeMillis();
        while (System.currentTimeMillis() - alignStart < 1500) {
            swerve.getFrontLeft().alignTurningOnly(0);
            swerve.getFrontRight().alignTurningOnly(0);
            swerve.getBackLeft().alignTurningOnly(0);
            swerve.getBackRight().alignTurningOnly(0);
        }

        swerve.getFrontLeft().enableSaving();
        swerve.getFrontRight().enableSaving();
        swerve.getBackLeft().enableSaving();
        swerve.getBackRight().enableSaving();
        swerve.stopModules();
    }

    private void applyToModules(SwerveModuleState state) {
        boolean fl = (testMode == 0 || testMode == 1);
        boolean fr = (testMode == 0 || testMode == 2);
        boolean bl = (testMode == 0 || testMode == 3);
        boolean br = (testMode == 0 || testMode == 4);

        if (fl) swerve.getFrontLeft().setDesiredState(state);
        else    swerve.getFrontLeft().alignTurningOnly(0);

        if (fr) swerve.getFrontRight().setDesiredState(state);
        else    swerve.getFrontRight().alignTurningOnly(0);

        if (bl) swerve.getBackLeft().setDesiredState(state);
        else    swerve.getBackLeft().alignTurningOnly(0);

        if (br) swerve.getBackRight().setDesiredState(state);
        else    swerve.getBackRight().alignTurningOnly(0);
    }

    private SwerveModule getMonitorModule() {
        switch (testMode) {
            case 2: return swerve.getFrontRight();
            case 3: return swerve.getBackLeft();
            case 4: return swerve.getBackRight();
            default: return swerve.getFrontLeft();
        }
    }

    private String getModeLabel() {
        switch (testMode) {
            case 1: return "FL 單獨";
            case 2: return "FR 單獨";
            case 3: return "BL 單獨";
            case 4: return "BR 單獨";
            default: return "全部四顆";
        }
    }
}