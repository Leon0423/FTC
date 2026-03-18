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

            // ★ 計算該輪次的目標角度（根據當前模式）
            double[] currentTargetAngles = null;
            if (angularMode) {
                ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1.0);
                SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
                currentTargetAngles = new double[4];
                currentTargetAngles[0] = states[0].angle.getRadians();
                currentTargetAngles[1] = states[1].angle.getRadians();
                currentTargetAngles[2] = states[2].angle.getRadians();
                currentTargetAngles[3] = states[3].angle.getRadians();
            } else {
                currentTargetAngles = new double[]{0, 0, 0, 0};
            }

            // ★ Init 階段持續對齊輪子到目標角度
            swerve.getFrontLeft().alignTurningOnly(currentTargetAngles[0]);
            swerve.getFrontRight().alignTurningOnly(currentTargetAngles[1]);
            swerve.getBackLeft().alignTurningOnly(currentTargetAngles[2]);
            swerve.getBackRight().alignTurningOnly(currentTargetAngles[3]);

            // 讀取目前角度顯示對齊狀態
            double flDeg = Math.toDegrees(swerve.getFrontLeft().getTurningPosition());
            double frDeg = Math.toDegrees(swerve.getFrontRight().getTurningPosition());
            double blDeg = Math.toDegrees(swerve.getBackLeft().getTurningPosition());
            double brDeg = Math.toDegrees(swerve.getBackRight().getTurningPosition());

            // ★ 計算預期的對齐角度
            double flTargetDeg = Math.toDegrees(currentTargetAngles[0]);
            double frTargetDeg = Math.toDegrees(currentTargetAngles[1]);
            double blTargetDeg = Math.toDegrees(currentTargetAngles[2]);
            double brTargetDeg = Math.toDegrees(currentTargetAngles[3]);

            boolean flOk = Math.abs(normalizeAngleDeg(flDeg - flTargetDeg)) < 5;
            boolean frOk = Math.abs(normalizeAngleDeg(frDeg - frTargetDeg)) < 5;
            boolean blOk = Math.abs(normalizeAngleDeg(blDeg - blTargetDeg)) < 5;
            boolean brOk = Math.abs(normalizeAngleDeg(brDeg - brTargetDeg)) < 5;
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
            telemetry.addData("FL", "%.1f° / %.1f°  %s", flDeg, flTargetDeg, flOk ? "✅" : "⏳");
            telemetry.addData("FR", "%.1f° / %.1f°  %s", frDeg, frTargetDeg, frOk ? "✅" : "⏳");
            telemetry.addData("BL", "%.1f° / %.1f°  %s", blDeg, blTargetDeg, blOk ? "✅" : "⏳");
            telemetry.addData("BR", "%.1f° / %.1f°  %s", brDeg, brTargetDeg, brOk ? "✅" : "⏳");
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

        double startTime = getRuntime();
        double lastTime  = startTime;
        double lastHeadingDeg = swerve.getHeading();

        // ★ 在主測試迴圈之前計算好角速度的輪子方向
        // ★ 【重要】不要用 normalize，避免 optimize 改變角度
        double[] targetAngles = null;
        if (angularMode) {
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1.0);
            SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
            // SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);  // ← 不用 normalize！
            targetAngles = new double[4];
            targetAngles[0] = states[0].angle.getRadians();
            targetAngles[1] = states[1].angle.getRadians();
            targetAngles[2] = states[2].angle.getRadians();
            targetAngles[3] = states[3].angle.getRadians();
        }

        // ══════════════════════════════
        //  主測試迴圈
        // ══════════════════════════════
        while (opModeIsActive()) {
            double now     = getRuntime();
            double elapsed = now - startTime;
            double dt      = now - lastTime;
            lastTime = now;

            if (elapsed >= testDurationSec) break;

            // ★ 每個迴圈都持續給命令
            if (angularMode) {
                swerve.getFrontLeft().alignTurningOnly(targetAngles[0]);
                swerve.getFrontRight().alignTurningOnly(targetAngles[1]);
                swerve.getBackLeft().alignTurningOnly(targetAngles[2]);
                swerve.getBackRight().alignTurningOnly(targetAngles[3]);
                swerve.getFrontLeft().setDriveMotorPowerDirect(targetPowerRatio);
                swerve.getFrontRight().setDriveMotorPowerDirect(targetPowerRatio);
                swerve.getBackLeft().setDriveMotorPowerDirect(targetPowerRatio);
                swerve.getBackRight().setDriveMotorPowerDirect(targetPowerRatio);
            } else {
                swerve.getFrontLeft().alignTurningOnly(0);
                swerve.getFrontRight().alignTurningOnly(0);
                swerve.getBackLeft().alignTurningOnly(0);
                swerve.getBackRight().alignTurningOnly(0);
                swerve.getFrontLeft().setDriveMotorPowerDirect(targetPowerRatio);
                swerve.getFrontRight().setDriveMotorPowerDirect(targetPowerRatio);
                swerve.getBackLeft().setDriveMotorPowerDirect(targetPowerRatio);
                swerve.getBackRight().setDriveMotorPowerDirect(targetPowerRatio);
            }

            double velAvg      = averageWheelVelocity();
            double imuOmegaDeg = getImuAngularVelocityDeg();
            maxVelMps          = Math.max(maxVelMps, Math.abs(velAvg));
            maxOmegaRadPerSec  = Math.max(maxOmegaRadPerSec, Math.toRadians(Math.abs(imuOmegaDeg)));
            double heading      = swerve.getHeading();
            double diffOmegaDeg = dt > 0.001 ? (heading - lastHeadingDeg) / dt : 0;
            lastHeadingDeg      = heading;

            telemetry.addLine("════ 測試進行中 ════");
            telemetry.addData("模式",  angularMode ? "角速度" : "直線速度");
            telemetry.addData("進度",  "%.1f / %.1f 秒", elapsed, testDurationSec);
            telemetry.addLine("");

            // ★ 加這段：四顆個別速度
            telemetry.addLine("── 各顆速度 ──");
            telemetry.addData("FL", "%.3f m/s  (%.0f RPM)",
                    swerve.getFrontLeft().getDriveVelocity(),
                    swerve.getFrontLeft().getDriveRPM());
            telemetry.addData("FR", "%.3f m/s  (%.0f RPM)",
                    swerve.getFrontRight().getDriveVelocity(),
                    swerve.getFrontRight().getDriveRPM());
            telemetry.addData("BL", "%.3f m/s  (%.0f RPM)",
                    swerve.getBackLeft().getDriveVelocity(),
                    swerve.getBackLeft().getDriveRPM());
            telemetry.addData("BR", "%.3f m/s  (%.0f RPM)",
                    swerve.getBackRight().getDriveVelocity(),
                    swerve.getBackRight().getDriveRPM());
            telemetry.addLine("");

            if (angularMode) {
                telemetry.addData("當前角速度",        "%.3f rad/s", Math.toRadians(imuOmegaDeg));
                telemetry.addData("差分角速度（備用）", "%.1f deg/s", diffOmegaDeg);
                telemetry.addData("★ 最大角速度",      "%.3f rad/s", maxOmegaRadPerSec);
            } else {
                telemetry.addData("平均輪速",  "%.3f m/s", velAvg);
                telemetry.addData("★ 最大輪速", "%.3f m/s", maxVelMps);
            }
            telemetry.update();
            sleep(20);
        }

        // ══════════════════════════════
        //  結束：停車 + 清除角度記憶
        // ══════════════════════════════
        // 先停驅動馬達
        swerve.getFrontLeft().setDriveMotorPowerDirect(0);
        swerve.getFrontRight().setDriveMotorPowerDirect(0);
        swerve.getBackLeft().setDriveMotorPowerDirect(0);
        swerve.getBackRight().setDriveMotorPowerDirect(0);

        // 對齊回 0°，等到全部 < 5° 才存
        long alignStart = System.currentTimeMillis();
        while (System.currentTimeMillis() - alignStart < 2000 && opModeIsActive()) {
            swerve.getFrontLeft().alignTurningOnly(0);
            swerve.getFrontRight().alignTurningOnly(0);
            swerve.getBackLeft().alignTurningOnly(0);
            swerve.getBackRight().alignTurningOnly(0);

            double flDeg = Math.toDegrees(swerve.getFrontLeft().getTurningPosition());
            double frDeg = Math.toDegrees(swerve.getFrontRight().getTurningPosition());
            double blDeg = Math.toDegrees(swerve.getBackLeft().getTurningPosition());
            double brDeg = Math.toDegrees(swerve.getBackRight().getTurningPosition());

            if (Math.abs(flDeg) < 5 && Math.abs(frDeg) < 5
                    && Math.abs(blDeg) < 5 && Math.abs(brDeg) < 5) break;
        }

        // 全部對齊後才存
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

    /**
     * 將角度標準化到 [-180, 180] 度
     */
    private double normalizeAngleDeg(double angleDeg) {
        while (angleDeg >  180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}

