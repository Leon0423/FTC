package org.firstinspires.ftc.teamcode.subsystems;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final IMU imu;
    private final SwerveDriveOdometry odometer;

    // GoBILDA Pinpoint (可選)
    private GoBildaPinpointDriver pinpoint;
    private final boolean usingPinpoint;

    // Target position 追蹤
    private Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(0));

    // Dashboard 實例
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final double kStopSpeedMps = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.005; // ~0.5% of max

    private double headingOffset = 0;

    public SwerveSubsystem(HardwareMap hardwareMap) {

        frontLeft = new SwerveModule(
                hardwareMap,
                DriveConstants.kFrontLeftDriveMotorName,
                DriveConstants.kFrontLeftTurningMotorName,
                DriveConstants.kFrontLeftAbsoluteEncoderName,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
                DriveConstants.kFrontLeftDrivePowerScale);

        frontRight = new SwerveModule(
                hardwareMap,
                DriveConstants.kFrontRightDriveMotorName,
                DriveConstants.kFrontRightTurningMotorName,
                DriveConstants.kFrontRightAbsoluteEncoderName,
                DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
                DriveConstants.kFrontRightDrivePowerScale);


        backLeft = new SwerveModule(
                hardwareMap,
                DriveConstants.kBackLeftDriveMotorName,
                DriveConstants.kBackLeftTurningMotorName,
                DriveConstants.kBackLeftAbsoluteEncoderName,
                DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
                DriveConstants.kBackLeftDrivePowerScale);

        backRight = new SwerveModule(
                hardwareMap,
                DriveConstants.kBackRightDriveMotorName,
                DriveConstants.kBackRightTurningMotorName,
                DriveConstants.kBackRightAbsoluteEncoderName,
                DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
                DriveConstants.kBackRightDrivePowerScale);

        // 使用傳入的 hardwareMap
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        DriveConstants.kImuLogoFacingDirection,
                        DriveConstants.kImuUsbFacingDirection
                )
        );
        imu.initialize(parameters);

        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));

        // === Pinpoint 初始化 ===
        usingPinpoint = DriveConstants.USING_PINPOINT;
        if (usingPinpoint) {
            try {
                pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DriveConstants.kPinpointName);
                configurePinpoint();
            } catch (Exception e) {
                // 如果 Pinpoint 初始化失敗，記錄錯誤但不中斷
                pinpoint = null;
            }
        }

        // 不再在背景自動歸零；如需歸零請在 OpMode 內明確呼叫 zeroHeading()
    }

    /**
     * 配置 GoBILDA Pinpoint
     */
    private void configurePinpoint() {
        if (pinpoint == null) return;

        // 設定 Odometry Pod 的偏移量
        pinpoint.setOffsets(
                DriveConstants.kPinpointXPodOffsetMM,
                DriveConstants.kPinpointYPodOffsetMM,
                DistanceUnit.MM
        );

        // 設定 Odometry Pod 類型 (使用 goBILDA 4-Bar Pod)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // 設定編碼器方向
        pinpoint.setEncoderDirections(
                DriveConstants.kPinpointXEncoderReversed ?
                        GoBildaPinpointDriver.EncoderDirection.REVERSED :
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                DriveConstants.kPinpointYEncoderReversed ?
                        GoBildaPinpointDriver.EncoderDirection.REVERSED :
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // 重置位置和 IMU
        pinpoint.resetPosAndIMU();
    }

    // 修改 zeroHeading()
    public void zeroHeading() {
        if (usingPinpoint && pinpoint != null) {
            pinpoint.resetPosAndIMU();
        } else {
            headingOffset = Math.IEEEremainder(
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 360);
        }
    }

    // 修改 getHeading()
    public double getHeading() {
        if (usingPinpoint && pinpoint != null) {
            return Math.IEEEremainder(pinpoint.getHeading(AngleUnit.DEGREES), 360);
        } else {
            double raw = Math.IEEEremainder(
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 360);
            return Math.IEEEremainder(raw - headingOffset, 360);
        }
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        if (usingPinpoint && pinpoint != null) {
            Pose2D pose = pinpoint.getPosition();
            return new Pose2d(
                    pose.getX(DistanceUnit.METER),
                    pose.getY(DistanceUnit.METER),
                    Rotation2d.fromDegrees(pose.getHeading(AngleUnit.DEGREES))
            );
        } else {
            return odometer.getPoseMeters();
        }
    }

    public void resetOdometry(Pose2d pose) {
        if (usingPinpoint && pinpoint != null) {
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.METER,
                    pose.getX(),
                    pose.getY(),
                    AngleUnit.DEGREES,
                    pose.getRotation().getDegrees()
            ));
        } else {
            odometer.resetPosition(pose, getRotation2d());
        }
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose2d target) {
        this.targetPose = target;
    }

    public void setTargetPose(double x, double y, double headingDegrees) {
        this.targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
    }

    /**
     * 讓所有模組對齊到 0 rad（車頭方向），速度給極小值避免 setDesiredState 早停。
     */

    @Override
    public void periodic() {
        if (usingPinpoint && pinpoint != null) {
            // Pinpoint 需要每次迴圈呼叫 update() 來更新位置
            pinpoint.update();
        } else {
            // 使用原本的 SwerveDriveOdometry 更新位置
            odometer.updateWithTime(System.currentTimeMillis() / 1000.0, getRotation2d(),
                    frontLeft.getState(), frontRight.getState(),
                    backLeft.getState(), backRight.getState());
        }
    }

    public void updateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        // 發送數據到 Dashboard 圖表和場地視圖
        sendDashboardTelemetry();
    }

    /**
     * 發送位置數據到 FTC Dashboard 的圖表和繪圖區
     */
    public void sendDashboardTelemetry() {
        Pose2d currentPose = getPose();
        Pose2d target = getTargetPose();

        TelemetryPacket packet = new TelemetryPacket();

        // ===== 數據圖表 (Graph) =====
        // 位置數據
        packet.put("currentX", currentPose.getX());
        packet.put("currentY", currentPose.getY());
        packet.put("currentHeading", currentPose.getRotation().getDegrees());

        packet.put("targetX", target.getX());
        packet.put("targetY", target.getY());
        packet.put("targetHeading", target.getRotation().getDegrees());


        // ===== 場地繪圖 (Field) =====
        Canvas fieldOverlay = packet.fieldOverlay();

        // 繪製目標位置 (綠色)
        fieldOverlay.setStroke("#00FF00");
        fieldOverlay.setStrokeWidth(1);
        drawRobot(fieldOverlay, target, "#00FF00");

        // 繪製當前位置 (藍色)
        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.setStrokeWidth(1);
        drawRobot(fieldOverlay, currentPose, "#3F51B5");

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * 在 Dashboard 場地上繪製機器人
     */
    private void drawRobot(Canvas canvas, Pose2d pose, String color) {
        // 機器人尺寸 (英寸轉換到場地座標)
        final double ROBOT_RADIUS = 9; // 英寸

        // 轉換公尺到英寸 (Dashboard 使用英寸)
        double x = pose.getX() * 39.3701;
        double y = pose.getY() * 39.3701;
        double heading = pose.getRotation().getRadians();

        canvas.setFill(color);
        canvas.fillCircle(x, y, ROBOT_RADIUS);

        // 繪製方向指示線
        canvas.setStroke("#FFFFFF");
        canvas.setStrokeWidth(2);
        canvas.strokeLine(
                x, y,
                x + ROBOT_RADIUS * Math.cos(heading),
                y + ROBOT_RADIUS * Math.sin(heading)
        );
    }

    public SwerveModule getFrontLeft() { return frontLeft; }
    public SwerveModule getFrontRight() { return frontRight; }
    public SwerveModule getBackLeft() { return backLeft; }
    public SwerveModule getBackRight() { return backRight; }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Swerve 驅動（無 Drive PID，直接功率控制）
     * 用於調試或手動控制時繞過 PID
     * @param xSpeed 前進速度 (-1 到 1)
     * @param ySpeed 平移速度 (-1 到 1)
     * @param rot 旋轉速度 (-1 到 1)
     */
    public void driveNoPID(double xSpeed, double ySpeed, double rot) {
        // 將輸入轉換為速度（用於運動學計算）
        double vx = xSpeed * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double vy = ySpeed * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double omega = rot * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

        // 使用運動學計算每個模組的狀態
        com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds chassisSpeeds =
                new com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds(vx, vy, omega);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 歸一化輪速
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        // 設定每個模組（使用無 PID 模式）
        frontLeft.setDesiredStateNoPID(moduleStates[0]);
        frontRight.setDesiredStateNoPID(moduleStates[1]);
        backLeft.setDesiredStateNoPID(moduleStates[2]);
        backRight.setDesiredStateNoPID(moduleStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // If commanded speeds are tiny, hold current angles and stop drive motors
        boolean allStopped = true;
        for (SwerveModuleState s : desiredStates) {
            if (Math.abs(s.speedMetersPerSecond) > kStopSpeedMps) {
                allStopped = false;
                break;
            }
        }
        if (allStopped) {
            holdAngles();
            return;
        }

        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    private void holdAngles() {
        frontLeft.setDesiredState(new SwerveModuleState(0, frontLeft.getState().angle));
        frontRight.setDesiredState(new SwerveModuleState(0, frontRight.getState().angle));
        backLeft.setDesiredState(new SwerveModuleState(0, backLeft.getState().angle));
        backRight.setDesiredState(new SwerveModuleState(0, backRight.getState().angle));
    }

    private void addModuleTurningTelemetry(String name, SwerveModule module, Telemetry telemetry) {
        telemetry.addData(name + " Turning", "target:%.2f cur:%.2f err:%.2f out:%.2f",
                module.getTargetAngleRad(),
                module.getCurrentAngleRad(),
                module.getTurningErrorRad(),
                module.getTurningOutput());
    }

    public void resetAllModuleTracking() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    private void applyDriveReverseFromPrefs() {
        SharedPreferences prefs = AppUtil.getInstance().getRootActivity()
                .getSharedPreferences("SwerveDrivePrefs", Context.MODE_PRIVATE);

        // 如果有存過，覆蓋 Constants 的設定
        applyReverse(frontLeft,  DriveConstants.kFrontLeftTurningMotorName,  prefs);
        applyReverse(frontRight, DriveConstants.kFrontRightTurningMotorName, prefs);
        applyReverse(backLeft,   DriveConstants.kBackLeftTurningMotorName,   prefs);
        applyReverse(backRight,  DriveConstants.kBackRightTurningMotorName,  prefs);
    }

    private void applyReverse(SwerveModule module, String name, SharedPreferences prefs) {
        if (prefs.contains("drive_reverse_" + name)) {
            boolean reversed = prefs.getBoolean("drive_reverse_" + name, false);
            module.setDriveDirection(reversed);
        }
    }
}
