package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
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

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final IMU imu;
    private final SwerveDriveOdometry odometer;

    public SwerveSubsystem(HardwareMap hardwareMap) {

        frontLeft = new SwerveModule(
                hardwareMap,
                DriveConstants.kFrontLeftDriveMotorName,
                DriveConstants.kFrontLeftTurningMotorName,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        frontRight = new SwerveModule(
                hardwareMap,
                DriveConstants.kFrontRightDriveMotorName,
                DriveConstants.kFrontRightTurningMotorName,
                DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

        backLeft = new SwerveModule(
                hardwareMap,
                DriveConstants.kBackLeftDriveMotorName,
                DriveConstants.kBackLeftTurningMotorName,
                DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

        backRight = new SwerveModule(
                hardwareMap,
                DriveConstants.kBackRightDriveMotorName,
                DriveConstants.kBackRightTurningMotorName,
                DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderPort,
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

    public com.arcrobotics.ftclib.geometry.Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        odometer.updateWithTime(System.currentTimeMillis() / 1000.0, getRotation2d(),
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());
    }

    public void updateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        com.arcrobotics.ftclib.geometry.Pose2d pose = getPose();
        telemetry.addData("Robot Heading", getHeading());
        telemetry.addData("Robot Location", pose.getTranslation().toString());

        // Absolute Encoder readings
        telemetry.addData("", "=== Absolute Encoders ===");
        telemetry.addData("FL Abs Encoder (V)", "%.3f", frontLeft.getAbsoluteEncoderVoltage());
        telemetry.addData("FL Abs Encoder (Rad)", "%.3f", frontLeft.getAbsoluteEncoderRad());
        telemetry.addData("FL Abs Encoder (Deg)", "%.1f", Math.toDegrees(frontLeft.getAbsoluteEncoderRad()));

        telemetry.addData("FR Abs Encoder (V)", "%.3f", frontRight.getAbsoluteEncoderVoltage());
        telemetry.addData("FR Abs Encoder (Rad)", "%.3f", frontRight.getAbsoluteEncoderRad());
        telemetry.addData("FR Abs Encoder (Deg)", "%.1f", Math.toDegrees(frontRight.getAbsoluteEncoderRad()));

        telemetry.addData("BL Abs Encoder (V)", "%.3f", backLeft.getAbsoluteEncoderVoltage());
        telemetry.addData("BL Abs Encoder (Rad)", "%.3f", backLeft.getAbsoluteEncoderRad());
        telemetry.addData("BL Abs Encoder (Deg)", "%.1f", Math.toDegrees(backLeft.getAbsoluteEncoderRad()));

        telemetry.addData("BR Abs Encoder (V)", "%.3f", backRight.getAbsoluteEncoderVoltage());
        telemetry.addData("BR Abs Encoder (Rad)", "%.3f", backRight.getAbsoluteEncoderRad());
        telemetry.addData("BR Abs Encoder (Deg)", "%.1f", Math.toDegrees(backRight.getAbsoluteEncoderRad()));
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
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}