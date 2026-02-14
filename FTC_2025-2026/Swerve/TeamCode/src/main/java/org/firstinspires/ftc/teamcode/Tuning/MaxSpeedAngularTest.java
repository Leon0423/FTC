package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * 實測最大直線速度與角速度的 TeleOp。
 *
 * 使用方式：
 * 1. 在 Dashboard Config 調整 angularMode（true=角速度測試，false=直線測試）
 * 2. Init 後車輪會自動轉到對應測試的方向
 * 3. 按 Start 開始測試，測試時間到自動停止
 * 4. 查看 telemetry 的 Max 值作為實測結果
 *
 * 直線模式：全部輪子朝前，測平均輪速
 * 角速度模式：輪子呈 X 型，測 IMU 角速度
 */
@Config
@TeleOp(name = "MaxSpeedAngularTest", group = "Tuning")
public class MaxSpeedAngularTest extends LinearOpMode {

    // === Dashboard 可調參數（Init 前設定）===
    public static boolean angularMode = false;          // false: 直線速度；true: 角速度
    public static double targetPowerRatio = 1.0;        // 測試功率比例 0~1（1.0=全油門）
    public static double testDurationSec = 3.0;         // 測試維持時間 (秒)

    private SwerveSubsystem swerve;
    private IMU imu;

    // 記錄最大值
    private double maxVelMps = 0;
    private double maxOmegaDegPerSec = 0;
    private double maxOmegaRadPerSec = 0;

    // 差分用（備用）
    private double lastHeadingDeg = 0;
    private double lastTime = 0;

    @Override
    public void runOpMode() {
        // 初始化
        swerve = new SwerveSubsystem(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ===== INIT 階段：持續對齊車輪直到按下 Start =====
        while (!isStarted() && !isStopRequested()) {
            // 持續對齊車輪
            alignWheelsForTest();

            telemetry.addLine("=== 最大速度/角速度 測試 ===");
            telemetry.addData("測試模式", angularMode ? "角速度 (旋轉)" : "直線速度");
            telemetry.addData("功率比例", "%.0f%%", targetPowerRatio * 100);
            telemetry.addData("測試時間", "%.1f 秒", testDurationSec);
            telemetry.addLine("");
            telemetry.addLine(">>> 車輪校正中，按 Start 開始測試 <<<");
            telemetry.update();

            sleep(20);
        }

        if (!opModeIsActive()) return;

        // 重置紀錄
        maxVelMps = 0;
        maxOmegaDegPerSec = 0;
        maxOmegaRadPerSec = 0;
        lastHeadingDeg = swerve.getHeading();
        lastTime = getRuntime();
        double startTime = getRuntime();

        // ===== 主測試迴圈 =====
        while (opModeIsActive()) {
            double now = getRuntime();
            double elapsed = now - startTime;
            double dt = now - lastTime;
            lastTime = now;

            // 檢查是否超時
            if (elapsed >= testDurationSec) {
                break;
            }

            // 根據模式下達命令
            if (angularMode) {
                // 角速度測試：原地旋轉
                runAngularTest();
            } else {
                // 直線測試：向前全速
                runLinearTest();
            }

            // ===== 讀取並記錄數據 =====

            // 1. 平均輪速（直線用）
            double velAvg = averageWheelVelocity();
            maxVelMps = Math.max(maxVelMps, Math.abs(velAvg));

            // 2. IMU 角速度（直接讀取，更準確）
            double imuOmegaDeg = getImuAngularVelocityDeg();
            maxOmegaDegPerSec = Math.max(maxOmegaDegPerSec, Math.abs(imuOmegaDeg));
            maxOmegaRadPerSec = Math.toRadians(maxOmegaDegPerSec);

            // 3. 差分角速度（備用驗證）
            double heading = swerve.getHeading();
            double diffOmegaDeg = (dt > 0.001) ? (heading - lastHeadingDeg) / dt : 0;
            lastHeadingDeg = heading;

            // ===== Telemetry 顯示 =====
            telemetry.addLine("=== 測試進行中 ===");
            telemetry.addData("模式", angularMode ? "角速度" : "直線速度");
            telemetry.addData("進度", "%.1f / %.1f 秒", elapsed, testDurationSec);
            telemetry.addLine("");

            if (angularMode) {
                telemetry.addData("當前角速度 (rad/s)", "%.3f", Math.toRadians(imuOmegaDeg));
                telemetry.addData("★ kPhysicalMaxAngularSpeed", "%.3f rad/s", maxOmegaRadPerSec);
            } else {
                telemetry.addData("當前輪速 (m/s)", "%.3f", velAvg);
                telemetry.addData("★ kPhysicalMaxSpeed", "%.3f m/s", maxVelMps);
            }
            telemetry.update();

            sleep(20);
        }

        // ===== 測試結束 =====
        swerve.stopModules();

        // 顯示最終結果
        while (opModeIsActive()) {
            telemetry.addLine("========== 測試完成 ==========");
            telemetry.addData("測試模式", angularMode ? "角速度" : "直線速度");
            telemetry.addLine("");
            telemetry.addLine(">>> 請將以下數值填入 Constants.java <<<");
            if (angularMode) {
                telemetry.addData("kPhysicalMaxAngularSpeedRadiansPerSecond", "%.3f", maxOmegaRadPerSec);
            } else {
                telemetry.addData("kPhysicalMaxSpeedMetersPerSecond", "%.3f", maxVelMps);
            }
            telemetry.update();
            sleep(100);
        }
    }

    /**
     * 根據測試模式預先對齊車輪
     */
    private void alignWheelsForTest() {
        if (angularMode) {
            // 角速度測試：輪子呈 X 型（各模組 45 度角指向旋轉切線）
            // 前左 & 後右: +45度, 前右 & 後左: -45度
            double angle45 = Math.PI / 4;
            SwerveModuleState stateFLBR = new SwerveModuleState(0.001, new Rotation2d(angle45));
            SwerveModuleState stateFRBL = new SwerveModuleState(0.001, new Rotation2d(-angle45));

            swerve.getFrontLeft().setDesiredState(stateFLBR);
            swerve.getFrontRight().setDesiredState(stateFRBL);
            swerve.getBackLeft().setDesiredState(stateFRBL);
            swerve.getBackRight().setDesiredState(stateFLBR);
        } else {
            // 直線測試：所有輪子朝前（0 度）
            SwerveModuleState stateForward = new SwerveModuleState(0.001, new Rotation2d(0));
            swerve.getFrontLeft().setDesiredState(stateForward);
            swerve.getFrontRight().setDesiredState(stateForward);
            swerve.getBackLeft().setDesiredState(stateForward);
            swerve.getBackRight().setDesiredState(stateForward);
        }
    }

    /**
     * 執行直線速度測試
     */
    private void runLinearTest() {
        // 直接設定各模組：朝前 + 最大速度
        double speed = Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond * targetPowerRatio;
        SwerveModuleState state = new SwerveModuleState(speed, new Rotation2d(0));

        swerve.getFrontLeft().setDesiredState(state);
        swerve.getFrontRight().setDesiredState(state);
        swerve.getBackLeft().setDesiredState(state);
        swerve.getBackRight().setDesiredState(state);
    }

    /**
     * 執行角速度測試
     */
    private void runAngularTest() {
        // 使用 ChassisSpeeds 計算原地旋轉的各輪狀態
        double omega = Constants.DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * targetPowerRatio;
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);
        SwerveModuleState[] states = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        swerve.setModuleStates(states);
    }

    /**
     * 從 IMU 直接讀取角速度 (deg/s)
     */
    private double getImuAngularVelocityDeg() {
        try {
            AngularVelocity angVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            return angVel.zRotationRate;  // Z 軸為 yaw 旋轉
        } catch (Exception e) {
            return 0;
        }
    }

    /**
     * 計算四輪平均速度
     */
    private double averageWheelVelocity() {
        SwerveModule fl = swerve.getFrontLeft();
        SwerveModule fr = swerve.getFrontRight();
        SwerveModule bl = swerve.getBackLeft();
        SwerveModule br = swerve.getBackRight();
        return (fl.getDriveVelocity() + fr.getDriveVelocity() +
                bl.getDriveVelocity() + br.getDriveVelocity()) / 4.0;
    }
}
