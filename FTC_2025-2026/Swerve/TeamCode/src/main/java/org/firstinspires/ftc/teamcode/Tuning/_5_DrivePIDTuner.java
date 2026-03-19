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
 * 調整順序：
 *   Step 1：只用 F，調到速度大致跟上（P=I=D=0）
 *   Step 2：加 P 修正誤差（從 0.01 開始）
 *   Step 3：如果震盪，降 P 或加 D
 *   Step 4：有穩態誤差才加 I（通常不需要）
 *
 * 如果不管怎麼調都震盪：
 *   → 只用 F，不用 PID
 *   → 在 TuningConfig 把 enableDrivePID 設 false
 */
@Config
@TeleOp(name = "5. Drive PID Tuner", group = "Tuning")
public class _5_DrivePIDTuner extends LinearOpMode {

    public static double autoTestPeriodSeconds = 2.0;
    public static double autoTestMaxVelocity   = 0.3;

    // ★ F-only 模式：只用前饋，完全不用 PID
    // 先調這個讓速度大致正確，再考慮要不要加 PID
    public static boolean fOnlyMode = true;
    public static double manualF = 0.5;  // F-only 模式下直接給這個比例的功率

    private SwerveSubsystem swerve;
    private FtcDashboard dashboard;

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
        //  Init 階段
        // ══════════════════════════════
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a)          testMode = 0;
            if (gamepad1.dpad_up)    testMode = 1;
            if (gamepad1.dpad_right) testMode = 2;
            if (gamepad1.dpad_left)  testMode = 3;
            if (gamepad1.dpad_down)  testMode = 4;

            swerve.getFrontLeft().alignTurningOnly(0);
            swerve.getFrontRight().alignTurningOnly(0);
            swerve.getBackLeft().alignTurningOnly(0);
            swerve.getBackRight().alignTurningOnly(0);

            telemetry.addLine("════ Drive PID Tuner ════");
            telemetry.addLine("A=全部  ↑=FL  →=FR  ←=BL  ↓=BR");
            telemetry.addData("目前選擇", getModeLabel());
            telemetry.addData("F-Only 模式", fOnlyMode ? "ON（推薦先用這個）" : "OFF（使用 PID）");
            telemetry.addLine("按 Start 開始");
            telemetry.update();
        }

        if (!opModeIsActive()) return;

        double startTime = getRuntime();

        // ══════════════════════════════
        //  主測試迴圈
        // ══════════════════════════════
        while (opModeIsActive()) {

            boolean x = gamepad1.x;
            if (x && !lastX) {
                isPaused = !isPaused;
                if (!isPaused) startTime = getRuntime();
            }
            lastX = x;

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

            // 方波（四段式）
            double elapsed = getRuntime() - startTime;
            double cycleTime = autoTestPeriodSeconds * 4;
            double cyclePos = elapsed % cycleTime;
            double targetVel;
            if (cyclePos < autoTestPeriodSeconds) {
                targetVel = autoTestMaxVelocity;
            } else if (cyclePos < autoTestPeriodSeconds * 2) {
                targetVel = 0;
            } else if (cyclePos < autoTestPeriodSeconds * 3) {
                targetVel = -autoTestMaxVelocity;
            } else {
                targetVel = 0;
            }

            // F-Only 模式功率
            double power;
            if (cyclePos < autoTestPeriodSeconds) {
                power = manualF;
            } else if (cyclePos < autoTestPeriodSeconds * 2) {
                power = 0;
            } else if (cyclePos < autoTestPeriodSeconds * 3) {
                power = -manualF;
            } else {
                power = 0;
            }

            if (fOnlyMode) {
                applyFOnly(power);
            } else {
                SwerveModuleState state = new SwerveModuleState(targetVel, new Rotation2d(0));
                applyToModules(state);
            }

            SwerveModule monitor = getMonitorModule();

            // 取得速度數據（根據模式選擇正確的讀取方式）
            // F-Only 模式下因為沒有呼叫 setDesiredState，必須主動呼叫 getDriveVelocity() 來更新濾波器並取得當前速度
            // PID 模式下 setDesiredState 已經呼叫過 getDriveVelocity()，所以直接讀取 getCurrentVelocityMps() 避免重複濾波
            double flVel, frVel, blVel, brVel;
            if (fOnlyMode) {
                flVel = swerve.getFrontLeft().getDriveVelocity();
                frVel = swerve.getFrontRight().getDriveVelocity();
                blVel = swerve.getBackLeft().getDriveVelocity();
                brVel = swerve.getBackRight().getDriveVelocity();
            } else {
                flVel = swerve.getFrontLeft().getCurrentVelocityMps();
                frVel = swerve.getFrontRight().getCurrentVelocityMps();
                blVel = swerve.getBackLeft().getCurrentVelocityMps();
                brVel = swerve.getBackRight().getCurrentVelocityMps();
            }

            // Dashboard graph
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("targetVel",  targetVel);
            packet.put("FL_vel",     flVel);
            packet.put("FR_vel",     frVel);
            packet.put("BL_vel",     blVel);
            packet.put("BR_vel",     brVel);
            // ★ 同時顯示 raw 速度，判斷雜訊量
            packet.put("FL_raw",     swerve.getFrontLeft().getRawDriveVelocity());
            if (!fOnlyMode) {
                packet.put("driveOutput", monitor.getDriveOutput());
                packet.put("driveError",  monitor.getDriveError());
            }
            dashboard.sendTelemetryPacket(packet);

            // Telemetry
            telemetry.addLine("════ Drive PID Tuner ════");
            telemetry.addData("模式", fOnlyMode ? "F-Only（直接功率）" : "PID");
            telemetry.addData("測試模組", getModeLabel());
            if (fOnlyMode) {
                telemetry.addData("功率", "%.2f  (Dashboard 調 manualF)", manualF);
            } else {
                telemetry.addData("P", "%.4f", TuningConfig.driveP());
                telemetry.addData("I", "%.4f", TuningConfig.driveI());
                telemetry.addData("D", "%.4f", TuningConfig.driveD());
                telemetry.addData("F", "%.4f", TuningConfig.driveF());
            }
            telemetry.addLine("");
            telemetry.addData("目標速度", "%.3f m/s", targetVel);
            telemetry.addLine("── filtered / raw ──");
            telemetry.addData("FL", "%.3f / %.3f m/s",
                    flVel,
                    swerve.getFrontLeft().getRawDriveVelocity());
            telemetry.addData("FR", "%.3f / %.3f m/s",
                    frVel,
                    swerve.getFrontRight().getRawDriveVelocity());
            telemetry.addData("BL", "%.3f / %.3f m/s",
                    blVel,
                    swerve.getBackLeft().getRawDriveVelocity());
            telemetry.addData("BR", "%.3f / %.3f m/s",
                    brVel,
                    swerve.getBackRight().getRawDriveVelocity());
            telemetry.addLine("X=暫停  LB/RB=調速度");
            telemetry.update();
        }

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

    // ★ F-Only：只給轉向對齊 + 直接功率，完全不走 PID
    private void applyFOnly(double power) {
        boolean fl = (testMode == 0 || testMode == 1);
        boolean fr = (testMode == 0 || testMode == 2);
        boolean bl = (testMode == 0 || testMode == 3);
        boolean br = (testMode == 0 || testMode == 4);

        swerve.getFrontLeft().alignTurningOnly(0);
        swerve.getFrontRight().alignTurningOnly(0);
        swerve.getBackLeft().alignTurningOnly(0);
        swerve.getBackRight().alignTurningOnly(0);

        swerve.getFrontLeft().setDriveMotorPowerDirect(fl ? power : 0);
        swerve.getFrontRight().setDriveMotorPowerDirect(fr ? power : 0);
        swerve.getBackLeft().setDriveMotorPowerDirect(bl ? power : 0);
        swerve.getBackRight().setDriveMotorPowerDirect(br ? power : 0);
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

