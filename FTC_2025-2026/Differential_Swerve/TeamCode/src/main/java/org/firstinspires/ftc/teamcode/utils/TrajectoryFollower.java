package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * 軌跡跟隨工具類別
 * 用於簡化自動模式中重複的軌跡跟隨邏輯
 */
public class TrajectoryFollower {

    /**
     * Heading Interpolation 模式
     */
    public enum HeadingMode {
        /** 機器人朝向跟隨路徑曲線方向（面向移動方向） */
        CURVE,
        /** 機器人朝向從起始角度線性插值到結束角度 */
        LINEAR,
        /** 機器人在整個路徑中保持固定朝向 */
        CONSTANT
    }

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveSubsystem swerveSubsystem;
    private final LinearOpMode opMode;
    private final FtcDashboard dashboard;

    // Dashboard 繪圖設定
    private static final int PATH_SAMPLE_COUNT = 50;  // 路徑取樣點數
    private static final double ROBOT_RADIUS = 0.1;   // 機器人半徑 (公尺)

    /**
     * 建構函式
     * @param swerveSubsystem Swerve 驅動子系統
     * @param opMode 當前的 OpMode (用於存取 telemetry 和 opModeIsActive)
     */
    public TrajectoryFollower(SwerveSubsystem swerveSubsystem, LinearOpMode opMode) {
        this.swerveSubsystem = swerveSubsystem;
        this.opMode = opMode;
        this.dashboard = FtcDashboard.getInstance();

        // 初始化 PID 控制器
        xController = new PIDController(
                AutoConstants.kPXController,
                AutoConstants.kIXController,
                AutoConstants.kDXController);
        yController = new PIDController(
                AutoConstants.kPYController,
                AutoConstants.kIYController,
                AutoConstants.kDYController);
        thetaController = new PIDController(
                AutoConstants.kPThetaController,
                AutoConstants.kIThetaController,
                AutoConstants.kDThetaController);
    }

    /**
     * 跟隨指定的軌跡（使用軌跡內建的朝向）
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱 (用於 telemetry 顯示)
     */
    public void followTrajectory(Trajectory trajectory, String phaseName) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.CURVE, 0, 0, 1.0, 0);
    }

    /**
     * 跟隨指定的軌跡（使用軌跡內建的朝向，可調速度）
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱 (用於 telemetry 顯示)
     * @param speedMultiplier 速度倍率 (0.0 ~ 1.0)
     */
    public void followTrajectory(Trajectory trajectory, String phaseName, double speedMultiplier) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.CURVE, 0, 0, clampSpeed(speedMultiplier), 0);
    }

    /**
     * 跟隨軌跡 - 使用曲線朝向 (Curve Heading Interpolation)
     * 機器人會面向移動方向
     *
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱
     */
    public void followTrajectoryCurveHeading(Trajectory trajectory, String phaseName) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.CURVE, 0, 0, 1.0, 0);
    }

    /**
     * 跟隨軌跡 - 使用曲線朝向 (Curve Heading Interpolation)
     * 機器人會面向移動方向
     *
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱
     * @param speedMultiplier 速度倍率 (0.0 ~ 1.0)
     */
    public void followTrajectoryCurveHeading(Trajectory trajectory, String phaseName, double speedMultiplier) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.CURVE, 0, 0, clampSpeed(speedMultiplier), 0);
    }

    /**
     * 跟隨軌跡 - 使用線性朝向插值 (Linear Heading Interpolation)
     * 機器人朝向會從 startHeadingDeg 線性變化到 endHeadingDeg
     *
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱
     * @param startHeadingDeg 起始朝向 (度)
     * @param endHeadingDeg 結束朝向 (度)
     */
    public void followTrajectoryLinearHeading(Trajectory trajectory, String phaseName,
                                               double startHeadingDeg, double endHeadingDeg) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.LINEAR,
                Math.toRadians(startHeadingDeg), Math.toRadians(endHeadingDeg), 1.0, 0);
    }

    /**
     * 跟隨軌跡 - 使用線性朝向插值 (Linear Heading Interpolation)
     * 機器人朝向會從 startHeadingDeg 線性變化到 endHeadingDeg
     *
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱
     * @param startHeadingDeg 起始朝向 (度)
     * @param endHeadingDeg 結束朝向 (度)
     * @param speedMultiplier 速度倍率 (0.0 ~ 1.0)
     */
    public void followTrajectoryLinearHeading(Trajectory trajectory, String phaseName,
                                               double startHeadingDeg, double endHeadingDeg,
                                               double speedMultiplier) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.LINEAR,
                Math.toRadians(startHeadingDeg), Math.toRadians(endHeadingDeg), clampSpeed(speedMultiplier), 0);
    }

    /**
     * 跟隨軌跡 - 使用線性朝向插值 (Linear Heading Interpolation)
     * 機器人朝向會從 startHeadingDeg 線性變化到 endHeadingDeg，並在路徑期間額外旋轉指定圈數
     *
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱
     * @param startHeadingDeg 起始朝向 (度)
     * @param endHeadingDeg 結束朝向 (度)
     * @param speedMultiplier 速度倍率 (0.0 ~ 1.0)
     * @param rotations 額外旋轉圈數 (正值=逆時針，負值=順時針)
     */
    public void followTrajectoryLinearHeading(Trajectory trajectory, String phaseName,
                                               double startHeadingDeg, double endHeadingDeg,
                                               double speedMultiplier, double rotations) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.LINEAR,
                Math.toRadians(startHeadingDeg), Math.toRadians(endHeadingDeg), clampSpeed(speedMultiplier), rotations);
    }

    /**
     * 跟隨軌跡 - 使用固定朝向 (Constant Heading Interpolation)
     * 機器人在整個路徑中保持固定朝向
     *
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱
     * @param constantHeadingDeg 固定朝向 (度)
     */
    public void followTrajectoryConstantHeading(Trajectory trajectory, String phaseName,
                                                 double constantHeadingDeg) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.CONSTANT,
                Math.toRadians(constantHeadingDeg), Math.toRadians(constantHeadingDeg), 1.0, 0);
    }

    /**
     * 跟隨軌跡 - 使用固定朝向 (Constant Heading Interpolation)
     * 機器人在整個路徑中保持固定朝向
     *
     * @param trajectory 要跟隨的軌跡
     * @param phaseName 階段名稱
     * @param constantHeadingDeg 固定朝向 (度)
     * @param speedMultiplier 速度倍率 (0.0 ~ 1.0)
     */
    public void followTrajectoryConstantHeading(Trajectory trajectory, String phaseName,
                                                 double constantHeadingDeg, double speedMultiplier) {
        followTrajectoryInternal(trajectory, phaseName, HeadingMode.CONSTANT,
                Math.toRadians(constantHeadingDeg), Math.toRadians(constantHeadingDeg), clampSpeed(speedMultiplier), 0);
    }

    /**
     * 內部軌跡跟隨實作
     * @param rotations 路徑期間額外旋轉的圈數 (僅 LINEAR 模式使用，正值=逆時針，負值=順時針)
     */
    private void followTrajectoryInternal(Trajectory trajectory, String phaseName,
                                          HeadingMode headingMode,
                                          double headingStart, double headingEnd,
                                          double speedMultiplier, double rotations) {
        // 自動重置 PID 控制器，避免積分累積
        resetControllers();

        // 預先計算路徑點用於 Dashboard 繪製
        double[][] pathPoints = sampleTrajectoryPath(trajectory);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // 原始軌跡時間
        double originalTotalTime = trajectory.getTotalTimeSeconds();
        // 根據速度倍率調整實際執行時間：speedMultiplier 越小，執行時間越長
        double adjustedTotalTime = originalTotalTime / speedMultiplier;

        while (opMode.opModeIsActive() && timer.seconds() < adjustedTotalTime) {
            double currentTime = timer.seconds();
            double progress = currentTime / adjustedTotalTime;  // 0.0 ~ 1.0

            // 將實際時間映射回軌跡時間進行取樣
            double trajectoryTime = currentTime * speedMultiplier;

            // 獲取當前時間的目標狀態
            Trajectory.State targetState = trajectory.sample(trajectoryTime);
            Pose2d targetPose = targetState.poseMeters;

            // 獲取當前機器人位置
            Pose2d currentPose = swerveSubsystem.getPose();

            // 計算位置 PID 輸出 (不縮放，讓 PID 正常追蹤)
            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());

            // 根據 HeadingMode 計算目標朝向 (LINEAR 模式支援額外旋轉圈數)
            double targetAngle = calculateTargetHeading(headingMode, targetState,
                                                        headingStart, headingEnd, progress, rotations);

            // 計算角度誤差 (處理角度跨越 -π 到 π 的情況)
            double currentAngle = currentPose.getRotation().getRadians();
            double angleError = normalizeAngle(targetAngle - currentAngle);
            double rotSpeed = thetaController.calculate(0, -angleError);

            // 加入前饋速度 (根據速度倍率縮放)
            xSpeed += targetState.velocityMetersPerSecond * Math.cos(targetState.poseMeters.getRotation().getRadians()) * speedMultiplier;
            ySpeed += targetState.velocityMetersPerSecond * Math.sin(targetState.poseMeters.getRotation().getRadians()) * speedMultiplier;

            // 轉換為 ChassisSpeeds 並計算模組狀態
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, currentPose.getRotation());

            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // 套用速度倍率限制：將最大允許速度降低
            double maxSpeed = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * speedMultiplier;
            com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, maxSpeed);

            swerveSubsystem.setModuleStates(moduleStates);

            // 更新里程計
            swerveSubsystem.periodic();

            // Dashboard 繪圖
            drawDashboard(pathPoints, currentPose, targetPose, targetAngle, phaseName, progress);

            // Telemetry
            opMode.telemetry.addData("Phase", phaseName);
            opMode.telemetry.addData("Heading Mode", headingMode.toString());
            opMode.telemetry.addData("Speed", "%.0f%%", speedMultiplier * 100);
            opMode.telemetry.addData("Progress", "%.1f%%", progress * 100);
            opMode.telemetry.addData("Time", "%.2f / %.2f", currentTime, adjustedTotalTime);
            opMode.telemetry.addData("Target", "X:%.2f Y:%.2f θ:%.1f°",
                    targetPose.getX(), targetPose.getY(), Math.toDegrees(targetAngle));
            opMode.telemetry.addData("Current", "X:%.2f Y:%.2f θ:%.1f°",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentAngle));
            opMode.telemetry.update();
        }

        // 軌跡完成後自動停止模組
        swerveSubsystem.stopModules();
    }

    /**
     * 將速度倍率限制在 0.0 ~ 1.0 範圍內
     * @param speed 輸入速度倍率
     * @return 限制後的速度倍率
     */
    private double clampSpeed(double speed) {
        return Math.max(0.0, Math.min(1.0, speed));
    }

    // ==================== Dashboard 繪圖方法 ====================

    /**
     * 取樣軌跡路徑點
     * @param trajectory 軌跡
     * @return 路徑點陣列 [x[], y[]]
     */
    private double[][] sampleTrajectoryPath(Trajectory trajectory) {
        double[] xPoints = new double[PATH_SAMPLE_COUNT];
        double[] yPoints = new double[PATH_SAMPLE_COUNT];
        double totalTime = trajectory.getTotalTimeSeconds();

        for (int i = 0; i < PATH_SAMPLE_COUNT; i++) {
            double t = (double) i / (PATH_SAMPLE_COUNT - 1) * totalTime;
            Pose2d pose = trajectory.sample(t).poseMeters;
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }

        return new double[][] { xPoints, yPoints };
    }

    /**
     * 繪製 Dashboard 畫面
     * 注意: Dashboard 的 Y 軸需要反轉，使 Y > 0 向右移動
     */
    private void drawDashboard(double[][] pathPoints, Pose2d currentPose, Pose2d targetPose,
                               double targetAngle, String phaseName, double progress) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // 設定座標系統 (以公尺為單位，放大顯示)
        double scale = 100.0;  // 1公尺 = 100像素

        // 繪製預測路徑 (藍色)
        canvas.setStroke("#3F51B5");
        canvas.setStrokeWidth(2);
        drawPath(canvas, pathPoints[0], pathPoints[1], scale);

        // 繪製目標位置 (綠色圓點)
        canvas.setFill("#4CAF50");
        canvas.fillCircle(targetPose.getX() * scale, -targetPose.getY() * scale, 5);

        // 繪製目標朝向箭頭 (綠色)
        canvas.setStroke("#4CAF50");
        canvas.setStrokeWidth(2);
        drawArrow(canvas, targetPose.getX() * scale, -targetPose.getY() * scale,
                  -targetAngle, ROBOT_RADIUS * scale * 1.5);

        // 繪製機器人當前位置 (紅色)
        canvas.setStroke("#F44336");
        canvas.setStrokeWidth(2);
        drawRobot(canvas, currentPose, scale);

        // 加入文字資訊
        packet.put("Phase", phaseName);
        packet.put("Progress", String.format("%.1f%%", progress * 100));
        packet.put("Current X", String.format("%.3f", currentPose.getX()));
        packet.put("Current Y", String.format("%.3f", currentPose.getY()));
        packet.put("Current θ", String.format("%.1f°", Math.toDegrees(currentPose.getRotation().getRadians())));
        packet.put("Target X", String.format("%.3f", targetPose.getX()));
        packet.put("Target Y", String.format("%.3f", targetPose.getY()));
        packet.put("Target θ", String.format("%.1f°", Math.toDegrees(targetAngle)));

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * 繪製路徑 (Y 軸反轉)
     */
    private void drawPath(Canvas canvas, double[] xPoints, double[] yPoints, double scale) {
        for (int i = 0; i < xPoints.length - 1; i++) {
            canvas.strokeLine(
                    xPoints[i] * scale, -yPoints[i] * scale,
                    xPoints[i + 1] * scale, -yPoints[i + 1] * scale
            );
        }
    }

    /**
     * 繪製機器人 (圓形 + 朝向箭頭，Y 軸反轉)
     */
    private void drawRobot(Canvas canvas, Pose2d pose, double scale) {
        double x = pose.getX() * scale;
        double y = -pose.getY() * scale;  // Y 軸反轉
        double heading = -pose.getRotation().getRadians();  // 朝向也需反轉
        double radius = ROBOT_RADIUS * scale;

        // 繪製機器人圓形外框
        canvas.strokeCircle(x, y, radius);

        // 繪製朝向箭頭
        drawArrow(canvas, x, y, heading, radius);
    }

    /**
     * 繪製箭頭
     */
    private void drawArrow(Canvas canvas, double x, double y, double heading, double length) {
        double endX = x + Math.cos(heading) * length;
        double endY = y + Math.sin(heading) * length;
        canvas.strokeLine(x, y, endX, endY);

        // 繪製箭頭頭部
        double arrowSize = length * 0.3;
        double arrowAngle = Math.toRadians(150);
        canvas.strokeLine(endX, endY,
                endX + Math.cos(heading + arrowAngle) * arrowSize,
                endY + Math.sin(heading + arrowAngle) * arrowSize);
        canvas.strokeLine(endX, endY,
                endX + Math.cos(heading - arrowAngle) * arrowSize,
                endY + Math.sin(heading - arrowAngle) * arrowSize);
    }

    /**
     * 根據 HeadingMode 計算目標朝向
     *
     * @param mode Heading 模式
     * @param state 當前軌跡狀態
     * @param startHeading 起始朝向 (用於 LINEAR 模式)
     * @param endHeading 結束朝向 (用於 LINEAR 模式)
     * @param progress 路徑進度 (0.0 ~ 1.0)
     * @param rotations 額外旋轉圈數 (僅 LINEAR 模式使用)
     * @return 目標朝向 (弧度)
     */
    private double calculateTargetHeading(HeadingMode mode, Trajectory.State state,
                                          double startHeading, double endHeading, double progress,
                                          double rotations) {
        switch (mode) {
            case CURVE:
                // 使用軌跡的曲線方向作為朝向
                return state.poseMeters.getRotation().getRadians();

            case LINEAR:
                // 從 startHeading 線性插值到 endHeading，加上額外旋轉圈數
                double angleDiff = normalizeAngle(endHeading - startHeading);
                double extraRotation = rotations * 2 * Math.PI * progress;
                return startHeading + angleDiff * progress + extraRotation;

            case CONSTANT:
                // 保持固定朝向
                return startHeading;

            default:
                return state.poseMeters.getRotation().getRadians();
        }
    }

    /**
     * 重置所有 PID 控制器 (避免積分累積)
     */
    public void resetControllers() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    /**
     * 將角度正規化到 -π 到 π 的範圍
     * @param angle 輸入角度 (弧度)
     * @return 正規化後的角度 (弧度)
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}

