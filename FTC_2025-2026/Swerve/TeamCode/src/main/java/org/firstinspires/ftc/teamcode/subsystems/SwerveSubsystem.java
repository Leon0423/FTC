package org.firstinspires.ftc.teamcode.subsystems;

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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final IMU imu;
    private final SwerveDriveOdometry odometer;

    // Target position 追蹤
    private Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(0));

    // Dashboard 實例
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final double kStopSpeedMps = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.02; // ~2% of max

    public SwerveSubsystem(HardwareMap hardwareMap) {

        frontLeft = new SwerveModule(
                hardwareMap,
                DriveConstants.kFrontLeftDriveMotorName,
                DriveConstants.kFrontLeftTurningMotorName,
                DriveConstants.kFrontLeftAbsoluteEncoderName,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        frontRight = new SwerveModule(
                hardwareMap,
                DriveConstants.kFrontRightDriveMotorName,
                DriveConstants.kFrontRightTurningMotorName,
                DriveConstants.kFrontRightAbsoluteEncoderName,
                DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

        backLeft = new SwerveModule(
                hardwareMap,
                DriveConstants.kBackLeftDriveMotorName,
                DriveConstants.kBackLeftTurningMotorName,
                DriveConstants.kBackLeftAbsoluteEncoderName,
                DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

        backRight = new SwerveModule(
                hardwareMap,
                DriveConstants.kBackRightDriveMotorName,
                DriveConstants.kBackRightTurningMotorName,
                DriveConstants.kBackRightAbsoluteEncoderName,
                DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

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

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
        } catch (Exception e) {
            // 如果角度計算失敗，直接使用原始角度
            // Log the error for debugging purposes
        }
        }).start();
    }

    public void zeroHeading() {
        imu.resetYaw();
    }

    public double getHeading() {
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        return Math.IEEEremainder(robotOrientation.getYaw(AngleUnit.DEGREES), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
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

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    /**
     * 讓所有模組對齊到 0 rad（車頭方向），速度給極小值避免 setDesiredState 早停。
     */
    public void alignModulesToForward() {
        SwerveModuleState zero = new SwerveModuleState(0.001, new Rotation2d(0));
        frontLeft.setDesiredState(zero);
        frontRight.setDesiredState(zero);
        backLeft.setDesiredState(zero);
        backRight.setDesiredState(zero);
    }

    @Override
    public void periodic() {
        odometer.updateWithTime(System.currentTimeMillis() / 1000.0, getRotation2d(),
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());
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
}
