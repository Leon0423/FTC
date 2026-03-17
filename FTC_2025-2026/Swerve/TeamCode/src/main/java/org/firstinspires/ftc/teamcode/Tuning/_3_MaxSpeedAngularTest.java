package org.firstinspires.ftc.teamcode.Tuning;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * ════════════════════════════════════════
 *  3. 最大速度 / 角速度 測試
 * ════════════════════════════════════════
 *
 * 目的：實測機器人的最大直線速度與最大角速度
 *       將結果填入 Constants.java 供控制程式使用
 *
 * 使用步驟：
 *   Init  │ A 鍵 = 直線速度測試
 *          │ B 鍵 = 角速度測試
 *          │ DPAD UP / DOWN = 調整功率 ±10%
 *   Start  │ 自動跑 testDurationSec 秒後停止
 *   結束   │ telemetry 顯示建議填入 Constants 的數值
 *
 * 注意：此程式不會污染 SharedPreferences
 *       結束時自動清除累積角度記憶
 */
@Config
@TeleOp(name = "3. MaxSpeedAngularTest", group = "Tuning")
public class _3_MaxSpeedAngularTest extends LinearOpMode {

    public static double testDurationSec = 3.0;

    private boolean angularMode = false;
    private double  targetPowerRatio = 0.5;

    private SwerveSubsystem swerve;
    private IMU imu;

    private double maxVelMps        = 0;
    private double maxOmegaRadPerSec = 0;

    boolean lastLb = false, lastRb = false;

    @Override
    public void runOpMode() {
        swerve = new SwerveSubsystem(hardwareMap);
        imu    = hardwareMap.get(IMU.class, "imu");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ★ 防止測試期間的角度寫入污染 Swerve_Control
        swerve.getFrontLeft().disableSaving();
        swerve.getFrontRight().disableSaving();
        swerve.getBackLeft().disableSaving();
        swerve.getBackRight().disableSaving();

        // ══════════════════════════════
        //  Init 階段：選擇模式 / 調功率
        // ══════════════════════════════
        boolean lastUp = false, lastDown = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) angularMode = false;
            if (gamepad1.b) angularMode = true;

            if (gamepad1.dpad_up   && !lastUp)   targetPowerRatio = Math.min(1.0, targetPowerRatio + 0.1);
            if (gamepad1.dpad_down && !lastDown)  targetPowerRatio = Math.max(0.1, targetPowerRatio - 0.1);
            lastUp   = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            // ★ LB/RB 調整測試時間
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            if (rb && !lastRb) testDurationSec = Math.min(10.0, testDurationSec + 0.5);
            if (lb && !lastLb) testDurationSec = Math.max(1.0,  testDurationSec - 0.5);
            lastRb = rb; lastLb = lb;

            // telemetry 加一行
            telemetry.addData("測試時間", "%.1f 秒  (LB/RB 調整)", testDurationSec);

            // ★ Init 階段持續對齊輪子到目標角度
            if (angularMode) {
                // 角速度模式：輪子呈 X 型（45°）
                double angle45 = Math.PI / 4;
                swerve.getFrontLeft().setDesiredState(new SwerveModuleState(0.001, new Rotation2d( angle45)));
                swerve.getFrontRight().setDesiredState(new SwerveModuleState(0.001, new Rotation2d(-angle45)));
                swerve.getBackLeft().setDesiredState(new SwerveModuleState(0.001, new Rotation2d(-angle45)));
                swerve.getBackRight().setDesiredState(new SwerveModuleState(0.001, new Rotation2d( angle45)));
            } else {
                // 直線模式：輪子全部朝前（0°）
                SwerveModuleState forward = new SwerveModuleState(0.001, new Rotation2d(0));
                swerve.getFrontLeft().setDesiredState(forward);
                swerve.getFrontRight().setDesiredState(forward);
                swerve.getBackLeft().setDesiredState(forward);
                swerve.getBackRight().setDesiredState(forward);
            }

            // 讀取目前角度顯示對齊狀態
            double flDeg = Math.toDegrees(swerve.getFrontLeft().getTurningPosition());
            double frDeg = Math.toDegrees(swerve.getFrontRight().getTurningPosition());
            double blDeg = Math.toDegrees(swerve.getBackLeft().getTurningPosition());
            double brDeg = Math.toDegrees(swerve.getBackRight().getTurningPosition());

            double targetDeg = angularMode ? 45.0 : 0.0;
            boolean flOk = Math.abs(Math.abs(flDeg) - targetDeg) < 5;
            boolean frOk = Math.abs(Math.abs(frDeg) - targetDeg) < 5;
            boolean blOk = Math.abs(Math.abs(blDeg) - targetDeg) < 5;
            boolean brOk = Math.abs(Math.abs(brDeg) - targetDeg) < 5;
            boolean allOk = flOk && frOk && blOk && brOk;

            telemetry.addLine("════ 最大速度 / 角速度 測試 ════");
            telemetry.addLine("");
            telemetry.addLine("A 鍵 = 直線速度測試（輪子朝前）");
            telemetry.addLine("B 鍵 = 角速度測試（輪子呈 X 型）");
            telemetry.addLine("DPAD UP / DOWN = 調整功率");
            telemetry.addLine("");
            telemetry.addData("測試模式", angularMode ? "【角速度（旋轉）】" : "【直線速度】");
            telemetry.addData("功率比例", "%.0f%%", targetPowerRatio * 100);
            telemetry.addData("測試時間", "%.1f 秒", testDurationSec);
            telemetry.addLine("");
            telemetry.addLine("── 輪子對齊狀態 ──");
            telemetry.addData("FL", "%.1f°  %s", flDeg, flOk ? "✅" : "⏳");
            telemetry.addData("FR", "%.1f°  %s", frDeg, frOk ? "✅" : "⏳");
            telemetry.addData("BL", "%.1f°  %s", blDeg, blOk ? "✅" : "⏳");
            telemetry.addData("BR", "%.1f°  %s", brDeg, brOk ? "✅" : "⏳");
            telemetry.addLine("");
            telemetry.addLine(allOk ? "✅ 對齊完成，可以按 Start！" : "⏳ 對齊中...");
            telemetry.update();
        }

        if (!opModeIsActive()) return;

        // ══════════════════════════════
        //  Start：下達命令
        // ══════════════════════════════
        maxVelMps        = 0;
        maxOmegaRadPerSec = 0;

        if (angularMode) runAngularTest();
        else             runLinearTest();

        double startTime = getRuntime();
        double lastTime  = startTime;
        double lastHeadingDeg = swerve.getHeading();

        // ══════════════════════════════
        //  主測試迴圈
        // ══════════════════════════════
        while (opModeIsActive()) {
            double now     = getRuntime();
            double elapsed = now - startTime;
            double dt      = now - lastTime;
            lastTime = now;

            if (elapsed >= testDurationSec) break;

            // 記錄最大值
            double velAvg     = averageWheelVelocity();
            double imuOmegaDeg = getImuAngularVelocityDeg();

            maxVelMps         = Math.max(maxVelMps,        Math.abs(velAvg));
            maxOmegaRadPerSec = Math.max(maxOmegaRadPerSec, Math.toRadians(Math.abs(imuOmegaDeg)));

            // 差分角速度（備用驗證）
            double heading     = swerve.getHeading();
            double diffOmegaDeg = dt > 0.001 ? (heading - lastHeadingDeg) / dt : 0;
            lastHeadingDeg = heading;

            telemetry.addLine("════ 測試進行中 ════");
            telemetry.addData("模式",  angularMode ? "角速度" : "直線速度");
            telemetry.addData("進度",  "%.1f / %.1f 秒", elapsed, testDurationSec);
            telemetry.addLine("");
            if (angularMode) {
                telemetry.addData("當前角速度",        "%.3f rad/s", Math.toRadians(imuOmegaDeg));
                telemetry.addData("差分角速度（備用）", "%.1f deg/s", diffOmegaDeg);
                telemetry.addData("★ 最大角速度",      "%.3f rad/s", maxOmegaRadPerSec);
            } else {
                telemetry.addData("當前輪速",  "%.3f m/s", velAvg);
                telemetry.addData("★ 最大輪速", "%.3f m/s", maxVelMps);
            }
            telemetry.update();
            sleep(20);
        }

        // ══════════════════════════════
        //  結束：停車 + 清除角度記憶
        // ══════════════════════════════
        swerve.stopModules();

        // 先對齊輪子回 0°，再儲存正確角度
        long alignStart = System.currentTimeMillis();
        while (System.currentTimeMillis() - alignStart < 1500 && opModeIsActive()) {
            SwerveModuleState forward = new SwerveModuleState(0.001, new Rotation2d(0));
            swerve.getFrontLeft().setDesiredState(forward);
            swerve.getFrontRight().setDesiredState(forward);
            swerve.getBackLeft().setDesiredState(forward);
            swerve.getBackRight().setDesiredState(forward);
        }

        // 重新開啟儲存，存入 0° 角度
        swerve.getFrontLeft().enableSaving();
        swerve.getFrontRight().enableSaving();
        swerve.getBackLeft().enableSaving();
        swerve.getBackRight().enableSaving();
        swerve.stopModules();

        // 顯示結果
        while (opModeIsActive()) {
            telemetry.addLine("════ 測試完成 ════");
            telemetry.addData("測試模式", angularMode ? "角速度" : "直線速度");
            telemetry.addLine("");
            telemetry.addLine("請將以下數值填入 Constants.java：");
            if (angularMode) {
                telemetry.addData("kPhysicalMaxAngularSpeedRadiansPerSecond", "%.3f", maxOmegaRadPerSec);
            } else {
                telemetry.addData("kPhysicalMaxSpeedMetersPerSecond", "%.3f", maxVelMps);
            }
            telemetry.addLine("");
            telemetry.addLine("角度記憶已清除，可直接跑 Swerve_Control");
            telemetry.update();
            sleep(100);
        }
    }

    private void runLinearTest() {
        double speed = Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond * targetPowerRatio;
        SwerveModuleState state = new SwerveModuleState(speed, new Rotation2d(0));
        swerve.getFrontLeft().setDesiredState(state);
        swerve.getFrontRight().setDesiredState(state);
        swerve.getBackLeft().setDesiredState(state);
        swerve.getBackRight().setDesiredState(state);
    }

    private void runAngularTest() {
        // 只用 kinematics 算輪子方向，速度直接給 targetPowerRatio
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1.0);
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);

        // 方向用 kinematics 算的，速度用 targetPowerRatio 覆蓋
        swerve.getFrontLeft().alignTurningOnly(states[0].angle.getRadians());
        swerve.getFrontRight().alignTurningOnly(states[1].angle.getRadians());
        swerve.getBackLeft().alignTurningOnly(states[2].angle.getRadians());
        swerve.getBackRight().alignTurningOnly(states[3].angle.getRadians());

        swerve.getFrontLeft().setDriveMotorPowerDirect(targetPowerRatio);
        swerve.getFrontRight().setDriveMotorPowerDirect(targetPowerRatio);
        swerve.getBackLeft().setDriveMotorPowerDirect(targetPowerRatio);
        swerve.getBackRight().setDriveMotorPowerDirect(targetPowerRatio);
    }

    private double getImuAngularVelocityDeg() {
        try {
            AngularVelocity angVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            return angVel.zRotationRate;
        } catch (Exception e) { return 0; }
    }

    private double averageWheelVelocity() {
        SwerveModule fl = swerve.getFrontLeft(),  fr = swerve.getFrontRight();
        SwerveModule bl = swerve.getBackLeft(),   br = swerve.getBackRight();
        return (fl.getDriveVelocity() + fr.getDriveVelocity()
                + bl.getDriveVelocity() + br.getDriveVelocity()) / 4.0;
    }

    // ★ 清除測試期間累積的角度記憶，避免污染 Swerve_Control
    private void clearAnglePrefs() {
        SharedPreferences prefs = AppUtil.getInstance().getRootActivity()
                .getSharedPreferences("SwerveModulePrefs", Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        for (String name : new String[]{
                DriveConstants.kFrontLeftTurningMotorName,
                DriveConstants.kFrontRightTurningMotorName,
                DriveConstants.kBackLeftTurningMotorName,
                DriveConstants.kBackRightTurningMotorName}) {
            editor.remove("swerve_angle_" + name);
            editor.remove("swerve_angle_" + name + "_raw");
        }
        editor.apply();
    }
}