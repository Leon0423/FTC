package org.firstinspires.ftc.teamcode.subsystems;

import android.content.SharedPreferences;

import org.firstinspires.ftc.teamcode.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.ModuleConstants;

public class SwerveModule {

    private final DcMotorEx   driveMotor;
    private final CRServo     turningServo;
    private final AnalogInput turningEncoder;
    private final boolean     turningEncoderReversed;
    private final double      absoluteEncoderOffsetRad;
    private final double      drivePowerScale;

    private final PIDController turningPid;
    private final PIDController drivePid;
    private final String        moduleName;

    // ── Delta-accumulation state ─────────────────────────────────────────
    private double  lastRawAngleRad;
    private double  accumulatedWheelDeg;
    private boolean accumulatorInitialized = false;

    // ══════════════════════════════════════════════════════════════════════

    public SwerveModule(HardwareMap hardwareMap,
                        String driveMotorName,   boolean driveMotorReversed,
                        String turningServoName, boolean turningServoReversed,
                        String turningEncoderName, boolean turningEncoderReversed,
                        double absoluteEncoderOffsetRad,
                        double drivePowerScale) {

        this.moduleName              = driveMotorName;
        this.turningEncoderReversed  = turningEncoderReversed;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
        this.drivePowerScale         = drivePowerScale;

        driveMotor     = hardwareMap.get(DcMotorEx.class,   driveMotorName);
        turningServo   = hardwareMap.get(CRServo.class,     turningServoName);
        turningEncoder = hardwareMap.get(AnalogInput.class, turningEncoderName);

        driveMotor.setDirection(driveMotorReversed
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turningServo.setDirection(turningServoReversed
                ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turningPid = new PIDController(
                ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        drivePid = new PIDController(
                ModuleConstants.kPDrive, ModuleConstants.kIDrive, ModuleConstants.kDDrive);
    }

    // ── Drive encoder ────────────────────────────────────────────────────

    public double getDrivePosition() {
        return driveMotor.getCurrentPosition() * ModuleConstants.kDriveEncoderTick2Meter;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity() * ModuleConstants.kDriveEncoderTick2Meter;
    }

    public double getDriveRPM() {
        return driveMotor.getVelocity() / ModuleConstants.kDriveEncoderTicksPerRevolution * 60.0;
    }

    public void resetDriveEncoder() {
        driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ── Turning encoder — raw readings ───────────────────────────────────

    public double getRawServoRadians() {
        return (turningEncoder.getVoltage() / ModuleConstants.kTurningEncoderMaxVoltage) * 2.0 * Math.PI;
    }

    public double getAbsoluteEncoderVoltage() {
        return turningEncoder.getVoltage();
    }

    public double getActiveOffsetRad() {
        return absoluteEncoderOffsetRad;
    }

    // ── Turning encoder — delta accumulation ─────────────────────────────

    public void updateAccumulator() {
        double rawRad = getRawServoRadians();

        if (!accumulatorInitialized) {
            lastRawAngleRad = rawRad;
            accumulatorInitialized = true;
            return;
        }

        double delta = rawRad - lastRawAngleRad;
        if (delta >  Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;
        if (turningEncoderReversed) delta = -delta;

        accumulatedWheelDeg += Math.toDegrees(delta) * ModuleConstants.kTurningMotorGearRatio;
        lastRawAngleRad = rawRad;
    }

    /** Wheel angle in radians, wrapped to [-π, π]. */
    public double getTurningPosition() {
        return angleWrap(Math.toRadians(accumulatedWheelDeg));
    }

    /** Wheel angle in radians, continuous (not wrapped). */
    public double getAccumulatedWheelAngleRad() {
        return Math.toRadians(accumulatedWheelDeg);
    }

    public double getAccumulatedWheelDeg() {
        return accumulatedWheelDeg;
    }

    // ── Tracking initialisation ──────────────────────────────────────────

    public void initializeTracking() {
        lastRawAngleRad = getRawServoRadians();
        accumulatedWheelDeg = 0;
        accumulatorInitialized = true;
    }

    // ── Persistence (SharedPreferences) ──────────────────────────────────

    public void saveState(SharedPreferences.Editor editor) {
        editor.putFloat(moduleName + "_lastRawRad",  (float) getRawServoRadians());
        editor.putFloat(moduleName + "_accWheelDeg", (float) accumulatedWheelDeg);
    }

    public boolean loadState(SharedPreferences prefs) {
        if (prefs.contains(moduleName + "_lastRawRad")) {
            lastRawAngleRad     = prefs.getFloat(moduleName + "_lastRawRad",  0f);
            accumulatedWheelDeg = prefs.getFloat(moduleName + "_accWheelDeg", 0f);
            accumulatorInitialized = true;
            return true;
        }
        return false;
    }

    // ── Zeroing ──────────────────────────────────────────────────────────

    public boolean driveToZero() {
        double remainder = accumulatedWheelDeg % 360.0;
        if (remainder >  180) remainder -= 360;
        if (remainder < -180) remainder += 360;

        if (Math.abs(remainder) < ModuleConstants.kZeroToleranceDeg) {
            turningServo.setPower(0);
            return true;
        }

        double output = -ModuleConstants.kPTurning * Math.toRadians(remainder);
        output = Math.max(-1, Math.min(1, output));
        turningServo.setPower(output);
        return false;
    }

    public void resetAccumulator() {
        accumulatedWheelDeg = 0;
    }

    // ── State (for kinematics / odometry) ────────────────────────────────

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, Telemetry telemetry) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        // === Drive ===
        if (ModuleConstants.kEnableDrivePID) {
            double targetVel  = state.speedMetersPerSecond;
            double currentVel = getDriveVelocity();
            double pidOut = drivePid.calculate(currentVel, targetVel);
            double ffOut  = ModuleConstants.kFDrive * targetVel;
            double power  = (pidOut + ffOut) * ModuleConstants.kDriveOutputScale * drivePowerScale;
            driveMotor.setPower(Math.max(-1, Math.min(1, power)));
        } else {
            double power = state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.setPower(power * drivePowerScale);
        }

        // === Turning ===
        double error = angleWrap(state.angle.getRadians() - getTurningPosition());
        applyTurningOutput(error);

        if (telemetry != null) {
            telemetry.addData("Swerve[" + moduleName + "]", state.toString());
        }
    }

    public void setTurningAngle(double angleRad) {
        driveMotor.setPower(0);
        double error = angleWrap(angleRad - getTurningPosition());
        applyTurningOutput(error);
    }

    public void stop() {
        driveMotor.setPower(0);
        turningServo.setPower(0);
    }

    public String getModuleName() {
        return moduleName;
    }

    // ── Turning output helper ────────────────────────────────────────────

    private void applyTurningOutput(double errorRad) {
        double absErrorDeg = Math.abs(Math.toDegrees(errorRad));

        if (absErrorDeg < ModuleConstants.kTurningDeadbandDeg) {
            turningServo.setPower(0);
            return;
        }

        double output = turningPid.calculate(0, -errorRad);

        if (absErrorDeg > ModuleConstants.kTurningMinOutputThreshDeg
                && Math.abs(output) < ModuleConstants.kTurningMinOutput) {
            output = Math.copySign(ModuleConstants.kTurningMinOutput, output);
        }

        output *= ModuleConstants.kTurningOutputScale;
        turningServo.setPower(Math.max(-1, Math.min(1, output)));
    }

    // ── Math helper ──────────────────────────────────────────────────────

    private static double angleWrap(double rad) {
        rad = rad % (2 * Math.PI);
        if (rad >  Math.PI) rad -= 2 * Math.PI;
        if (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
