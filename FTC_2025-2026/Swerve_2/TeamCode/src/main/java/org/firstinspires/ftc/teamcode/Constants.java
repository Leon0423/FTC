package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class Constants {

    public static final class ModuleConstants{
        // === 機械參數 ===
        public static final double kWheelDiameterMeters = 0.058;
        public static final double kDriveMotorGearRatio = 0.113636;
        public static final double kDriveEncoderTicksPerRevolution = 28.0;
        public static final double kServoTurnsPerWheelTurn = 2.5;
        public static final double kTurningMotorGearRatio = 1.0 / kServoTurnsPerWheelTurn;

        // === 編碼器換算係數 ===
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderTick2Meter = kDriveEncoderRot2Meter / kDriveEncoderTicksPerRevolution;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

        // === 轉向 PID 控制器參數 ===
        public static final double kPTurning = 0.4;
        public static final double kITurning = 0.0;
        public static final double kDTurning = 0.0;
        public static final double kTurningOutputScale = 1.0;

        public static final double kTurningDeadbandDeg = 2.5;

        public static final double kTurningMinOutput = 0.03;
        public static final double kTurningMinOutputThreshDeg = 8.0;

        // Axon servo 內建絕對編碼器：0–3.3V 對應 0–360° (一圈 servo)
        public static final double kTurningEncoderMaxVoltage = 3.3;

        // 歸零容差 (度) — alignToZero 判定到位的閾值
        public static final double kZeroToleranceDeg = 1.0;

        // === 驅動 PID 控制器參數 ===
        public static final double kPDrive = 0.5;
        public static final double kIDrive = 0.0;
        public static final double kDDrive = 0.0;
        public static final double kFDrive = 1.6299;
        public static final double kDriveOutputScale = 1.0;
        public static final boolean kEnableDrivePID = true;
    }

    public static final class DriveConstants {
        // === 機器人幾何參數 ===
        public static final double kTrackWidth = 0.22;
        public static final double kWheelBase = 0.22;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        // === 驅動馬達硬體名稱 ===
        public static final String kFrontLeftDriveMotorName = "FL";
        public static final String kBackLeftDriveMotorName = "BL";
        public static final String kFrontRightDriveMotorName = "FR";
        public static final String kBackRightDriveMotorName = "BR";

        // === 轉向馬達硬體名稱 ===
        public static final String kFrontLeftTurningMotorName = "FLTurn";
        public static final String kBackLeftTurningMotorName = "BLTurn";
        public static final String kFrontRightTurningMotorName = "FRTurn";
        public static final String kBackRightTurningMotorName = "BRTurn";

        // === 驅動編碼器方向設定 ===
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;

        // === 轉向編碼器方向設定 ===
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        // === 絕對編碼器硬體名稱 ===
        public static final String kFrontLeftAbsoluteEncoderName = "FLEncoder";
        public static final String kFrontRightAbsoluteEncoderName = "FREncoder";
        public static final String kBackLeftAbsoluteEncoderName = "BLEncoder";
        public static final String kBackRightAbsoluteEncoderName = "BREncoder";

        // === 絕對編碼器方向設定 ===
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // === 絕對編碼器偏移角度 (度數) ===
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 0.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = 0.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = 0.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = 0.0;

        // === 絕對編碼器偏移角度 (弧度) - 自動計算 ===
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(kFrontLeftDriveAbsoluteEncoderOffsetDeg);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(kBackLeftDriveAbsoluteEncoderOffsetDeg);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(kFrontRightDriveAbsoluteEncoderOffsetDeg);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(kBackRightDriveAbsoluteEncoderOffsetDeg);

        // === 機器人物理性能極限 ===
        public static final double kPhysicalMaxSpeedMetersPerSecond = 0.764;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 10.879;

        // === 手動控制性能限制 ===
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        // 各輪功率補償
        public static final double kFrontLeftDrivePowerScale  = 1.00;
        public static final double kFrontRightDrivePowerScale = 1.00;
        public static final double kBackLeftDrivePowerScale   = 1.00;
        public static final double kBackRightDrivePowerScale  = 1.00;

        // === IMU 配置 ===
        public static final RevHubOrientationOnRobot.LogoFacingDirection kImuLogoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static final RevHubOrientationOnRobot.UsbFacingDirection kImuUsbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // === GoBILDA Pinpoint 配置 ===
        public static final boolean USING_PINPOINT = true;
        public static final String kPinpointName = "pinpoint";
        public static final double kPinpointXPodOffsetMM = 0.0;
        public static final double kPinpointYPodOffsetMM = 0.0;
        public static final boolean kPinpointXEncoderReversed = false;
        public static final boolean kPinpointYEncoderReversed = true;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        public static final double kPXController = 0.0;
        public static final double kIXController = 0.0;
        public static final double kDXController = 0.0;

        public static final double kPYController = 0.0;
        public static final double kIYController = 0.0;
        public static final double kDYController = 0.0;

        public static final double kPThetaController = 0.0;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0;

        public static final double kPositionToleranceMeters = 0.02;
        public static final double kAngleToleranceDegrees = 2.0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double kDeadband = 0.1;
    }
}
