package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * 自動 PID 調整程式 - 定點校正模式
 * 機器人會持續維持在原點位置，被推開後會自動校正回去
 *
 * 使用方式：
 * 1. 連接 FTC Dashboard (http://192.168.43.1:8080/dash)
 * 2. 在 Config > AutoPIDTuner 選擇測試模式 (0=X, 1=Y, 2=Theta, 3=ALL)
 * 3. 啟動程式後，用手推機器人離開原點
 * 4. 觀察機器人是否能校正回原點
 * 5. 在 Dashboard 即時調整 PID 參數直到滿意
 */
@Config
@TeleOp(name = "Auto PID Tuner", group = "Tuning")
public class AutoPIDTuner extends LinearOpMode {

    // ========== Dashboard 可調參數 ==========

    // 0. 測試模式選擇 (0=X_ONLY, 1=Y_ONLY, 2=THETA_ONLY, 3=ALL)
    public static int _0_testMode = 3;

    // 1. X 軸 PID 參數 (前後移動) - 目標永遠是 0
    public static double _1a_X_kP = AutoConstants.kPXController;
    public static double _1b_X_kI = AutoConstants.kIXController;
    public static double _1c_X_kD = AutoConstants.kDXController;

    // 2. Y 軸 PID 參數 (左右移動) - 目標永遠是 0
    public static double _2a_Y_kP = AutoConstants.kPYController;
    public static double _2b_Y_kI = AutoConstants.kIYController;
    public static double _2c_Y_kD = AutoConstants.kDYController;

    // 3. Theta 旋轉 PID 參數 - 目標永遠是 0
    public static double _3a_Theta_kP = AutoConstants.kPThetaController;
    public static double _3b_Theta_kI = AutoConstants.kIThetaController;
    public static double _3c_Theta_kD = AutoConstants.kDThetaController;

    // Dashboard
    private FtcDashboard dashboard;

    // PID 控制器
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    @Override
    public void runOpMode() {
        // 初始化 Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // 初始化 SwerveSubsystem
        SwerveSubsystem swerveSubsystem = new SwerveSubsystem(hardwareMap);

        // 初始化 PID 控制器
        xController = new PIDController(_1a_X_kP, _1b_X_kI, _1c_X_kD);
        yController = new PIDController(_2a_Y_kP, _2b_Y_kI, _2c_Y_kD);
        thetaController = new PIDController(_3a_Theta_kP, _3b_Theta_kI, _3c_Theta_kD);

        // 顯示初始化資訊
        telemetry.addLine("========== Auto PID Tuner ==========");
        telemetry.addLine("定點校正模式：機器人會維持在原點");
        telemetry.addLine("用手推機器人，觀察校正效果");
        telemetry.addLine();
        telemetry.addLine("測試模式: 0=X, 1=Y, 2=Theta, 3=ALL");
        telemetry.addData("當前模式", _0_testMode);
        telemetry.addLine();
        telemetry.addData("狀態", "準備就緒，按 START 開始");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 重置里程計到原點 - 這就是目標位置 (0, 0, 0)
        swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

        // 持續運行直到停止
        while (opModeIsActive()) {
            // 即時更新 PID 參數 (從 Dashboard)
            xController.setPID(_1a_X_kP, _1b_X_kI, _1c_X_kD);
            yController.setPID(_2a_Y_kP, _2b_Y_kI, _2c_Y_kD);
            thetaController.setPID(_3a_Theta_kP, _3b_Theta_kI, _3c_Theta_kD);

            // 更新里程計
            swerveSubsystem.periodic();

            // 獲取當前機器人位置
            Pose2d currentPose = swerveSubsystem.getPose();
            double currentX = currentPose.getX();
            double currentY = currentPose.getY();
            double currentTheta = currentPose.getRotation().getRadians();

            // 目標永遠是原點 (0, 0, 0)
            double targetX = 0;
            double targetY = 0;
            double targetTheta = 0;

            // 計算誤差 (目標 - 當前 = 0 - current = -current)
            double xError = targetX - currentX;
            double yError = targetY - currentY;
            double thetaError = normalizeAngle(targetTheta - currentTheta);

            // 計算 PID 輸出 (原始輸出)
            double xPIDOutput = 0;
            double yPIDOutput = 0;
            double thetaPIDOutput = 0;

            // 根據測試模式啟用對應的控制器
            if (_0_testMode == 0 || _0_testMode == 3) {  // X_ONLY or ALL
                xPIDOutput = xController.calculate(currentX, targetX);
            }
            if (_0_testMode == 1 || _0_testMode == 3) {  // Y_ONLY or ALL
                yPIDOutput = yController.calculate(currentY, targetY);
            }
            if (_0_testMode == 2 || _0_testMode == 3) {  // THETA_ONLY or ALL
                thetaPIDOutput = thetaController.calculate(currentTheta, targetTheta);
            }

            // 將 PID 輸出乘以最大速度，轉換為實際速度 (m/s 或 rad/s)
            double xSpeed = xPIDOutput * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            double ySpeed = yPIDOutput * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            double rotSpeed = thetaPIDOutput * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

            // 轉換為機器人座標系的速度 (不使用 field relative)
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);

            // ===== Dashboard 圖表數據 =====
            TelemetryPacket packet = new TelemetryPacket();

            // 1. X 軸數據
            packet.put("_1a_targetX", targetX);
            packet.put("_1b_currentX", currentX);
            packet.put("_1c_xError", xError);

            // 2. Y 軸數據
            packet.put("_2a_targetY", targetY);
            packet.put("_2b_currentY", currentY);
            packet.put("_2c_yError", yError);

            // 3. Theta 數據 (度)
            packet.put("_3a_targetTheta", Math.toDegrees(targetTheta));
            packet.put("_3b_currentTheta", Math.toDegrees(currentTheta));
            packet.put("_3c_thetaError", Math.toDegrees(thetaError));

            dashboard.sendTelemetryPacket(packet);

            // Telemetry
            String[] modeNames = {"X_ONLY", "Y_ONLY", "THETA_ONLY", "ALL"};
            String modeName = (_0_testMode >= 0 && _0_testMode <= 3) ? modeNames[_0_testMode] : "UNKNOWN";

            telemetry.addLine("========== Auto PID Tuner ==========");
            telemetry.addLine("★ 定點校正模式 - 用手推機器人測試 ★");
            telemetry.addData("模式", modeName);
            telemetry.addLine();

            telemetry.addLine("--- PID 參數 (即時更新) ---");
            telemetry.addData("X PID", "P=%.2f I=%.3f D=%.3f", _1a_X_kP, _1b_X_kI, _1c_X_kD);
            telemetry.addData("Y PID", "P=%.2f I=%.3f D=%.3f", _2a_Y_kP, _2b_Y_kI, _2c_Y_kD);
            telemetry.addData("θ PID", "P=%.2f I=%.3f D=%.3f", _3a_Theta_kP, _3b_Theta_kI, _3c_Theta_kD);
            telemetry.addLine();

            telemetry.addLine("--- 當前位置 (目標: 0, 0, 0°) ---");
            telemetry.addData("X", "%.3f m (誤差: %.3f)", currentX, xError);
            telemetry.addData("Y", "%.3f m (誤差: %.3f)", currentY, yError);
            telemetry.addData("θ", "%.1f° (誤差: %.1f°)", Math.toDegrees(currentTheta), Math.toDegrees(thetaError));
            telemetry.addLine();

            telemetry.addLine("--- PID 輸出 ---");
            telemetry.addData("X", "PID:%.3f → Speed:%.3f m/s", xPIDOutput, xSpeed);
            telemetry.addData("Y", "PID:%.3f → Speed:%.3f m/s", yPIDOutput, ySpeed);
            telemetry.addData("θ", "PID:%.3f → Speed:%.3f rad/s", thetaPIDOutput, rotSpeed);
            telemetry.addLine();

            // 計算最大速度用於參考
            double maxSpeed = Math.max(Math.abs(xSpeed), Math.abs(ySpeed));
            double stopThreshold = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.005;
            telemetry.addData("最大速度", "%.3f m/s (閾值: %.3f)", maxSpeed, stopThreshold);
            telemetry.addData("馬達狀態", maxSpeed > stopThreshold ? "✓ 應該會動" : "✗ 低於閾值");
            telemetry.update();
        }

        // 停止所有模組
        swerveSubsystem.stopModules();
    }

    /**
     * 將角度正規化到 -π 到 π 的範圍
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}

