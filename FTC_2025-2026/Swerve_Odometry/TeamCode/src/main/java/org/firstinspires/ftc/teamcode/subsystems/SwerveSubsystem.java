package org.firstinspires.ftc.teamcode.subsystems;

import android.content.SharedPreferences;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final IMU                 imu;
    private final SwerveDriveOdometry odometer;
    private final ElapsedTime         odometryTimer = new ElapsedTime();

    private Telemetry telemetry;

    private double targetX, targetY, targetHeadingDeg;

    // ══════════════════════════════════════════════════════════════════════

    public SwerveSubsystem(HardwareMap hardwareMap) {

        frontLeft = new SwerveModule(hardwareMap,
                DriveConstants.kFrontLeftDriveMotorName,    DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningMotorName,  DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftAbsoluteEncoderName,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDrivePowerScale);

        frontRight = new SwerveModule(hardwareMap,
                DriveConstants.kFrontRightDriveMotorName,    DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningMotorName,  DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightAbsoluteEncoderName,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontRightDrivePowerScale);

        backLeft = new SwerveModule(hardwareMap,
                DriveConstants.kBackLeftDriveMotorName,    DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningMotorName,  DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftAbsoluteEncoderName,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDrivePowerScale);

        backRight = new SwerveModule(hardwareMap,
                DriveConstants.kBackRightDriveMotorName,    DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningMotorName,  DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightAbsoluteEncoderName,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackRightDrivePowerScale);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.kImuLogoFacingDirection,
                DriveConstants.kImuUsbFacingDirection)));

        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));
        zeroHeading();
    }

    // ── Module accessors ─────────────────────────────────────────────────

    public SwerveModule getFrontLeft()  { return frontLeft;  }
    public SwerveModule getFrontRight() { return frontRight; }
    public SwerveModule getBackLeft()   { return backLeft;   }
    public SwerveModule getBackRight()  { return backRight;  }

    // ── IMU ──────────────────────────────────────────────────────────────

    public void zeroHeading() {
        imu.resetYaw();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // ── Odometry ─────────────────────────────────────────────────────────

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    // ── Periodic (call every loop) ───────────────────────────────────────

    @Override
    public void periodic() {
        frontLeft.updateAccumulator();
        frontRight.updateAccumulator();
        backLeft.updateAccumulator();
        backRight.updateAccumulator();

        odometer.updateWithTime(odometryTimer.seconds(), getRotation2d(),
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());
    }

    // ── Tracking initialisation ──────────────────────────────────────────

    public void resetAllModuleTracking(boolean forceFromAbsolute) {
        frontLeft.initializeTracking();
        frontRight.initializeTracking();
        backLeft.initializeTracking();
        backRight.initializeTracking();
    }

    public void resetAllModuleTracking() {
        resetAllModuleTracking(false);
    }

    // ── Persistence (SharedPreferences) ──────────────────────────────────

    public boolean loadModuleStates(SharedPreferences prefs) {
        boolean fl = frontLeft.loadState(prefs);
        boolean fr = frontRight.loadState(prefs);
        boolean bl = backLeft.loadState(prefs);
        boolean br = backRight.loadState(prefs);
        return fl && fr && bl && br;
    }

    public void saveModuleStates(SharedPreferences.Editor editor) {
        frontLeft.saveState(editor);
        frontRight.saveState(editor);
        backLeft.saveState(editor);
        backRight.saveState(editor);
    }

    // ── Zeroing ──────────────────────────────────────────────────────────

    public boolean alignAllModulesToZero() {
        boolean fl = frontLeft.driveToZero();
        boolean fr = frontRight.driveToZero();
        boolean bl = backLeft.driveToZero();
        boolean br = backRight.driveToZero();
        return fl && fr && bl && br;
    }

    public void resetAllAccumulators() {
        frontLeft.resetAccumulator();
        frontRight.resetAccumulator();
        backLeft.resetAccumulator();
        backRight.resetAccumulator();
    }

    // ── Drive control ────────────────────────────────────────────────────

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates,
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], telemetry);
        frontRight.setDesiredState(desiredStates[1], telemetry);
        backLeft.setDesiredState(desiredStates[2], telemetry);
        backRight.setDesiredState(desiredStates[3], telemetry);
    }

    public void setXLockPattern() {
        frontLeft.setTurningAngle(Math.toRadians(45));
        frontRight.setTurningAngle(Math.toRadians(-45));
        backLeft.setTurningAngle(Math.toRadians(-45));
        backRight.setTurningAngle(Math.toRadians(45));
    }

    // ── Dashboard helpers ────────────────────────────────────────────────

    public void setTargetPose(double x, double y, double headingDeg) {
        this.targetX = x;
        this.targetY = y;
        this.targetHeadingDeg = headingDeg;
    }

    public void updateTelemetry(Telemetry t) {
        this.telemetry = t;

        t.addData("Robot Heading", "%.1f", getHeading());
        Pose2d pose = getPose();
        t.addData("Robot Location", "X:%.2f Y:%.2f", pose.getX(), pose.getY());

        t.addData("FL wheel deg", "%.1f", frontLeft.getAccumulatedWheelDeg());
        t.addData("FR wheel deg", "%.1f", frontRight.getAccumulatedWheelDeg());
        t.addData("BL wheel deg", "%.1f", backLeft.getAccumulatedWheelDeg());
        t.addData("BR wheel deg", "%.1f", backRight.getAccumulatedWheelDeg());
    }
}
