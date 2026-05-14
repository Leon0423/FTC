package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveModuleState;

import static org.firstinspires.ftc.teamcode.Constants.DIFFERENTIAL_DRIVE_POWER_SCALE;
import static org.firstinspires.ftc.teamcode.Constants.DIFFERENTIAL_STEER_POWER_SCALE;
import static org.firstinspires.ftc.teamcode.Constants.MAX_DRIVE_SPEED_MPS;
import static org.firstinspires.ftc.teamcode.Constants.MODULE_OPTIMIZE_MIN_SPEED_MPS;
import static org.firstinspires.ftc.teamcode.Constants.RESET_ENCODERS_ON_INIT;
import static org.firstinspires.ftc.teamcode.Constants.STEER_KD;
import static org.firstinspires.ftc.teamcode.Constants.STEER_KI;
import static org.firstinspires.ftc.teamcode.Constants.STEER_KP;
import static org.firstinspires.ftc.teamcode.Constants.STEER_TOLERANCE_RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.TICKS_TO_METERS_DRIVE;
import static org.firstinspires.ftc.teamcode.Constants.TICKS_TO_RADIANS_STEER;

@SuppressWarnings("deprecation")
public class SwerveModule {
    private final MotorEx motorA;
    private final MotorEx motorB;
    private final PIDController steerPID;

    public SwerveModule(HardwareMap hardwareMap,
                        String motorAName,
                        String motorBName,
                        boolean invertA,
                        boolean invertB) {
        motorA = new MotorEx(hardwareMap, motorAName);
        motorB = new MotorEx(hardwareMap, motorBName);

        motorA.setInverted(invertA);
        motorB.setInverted(invertB);

        motorA.setRunMode(Motor.RunMode.RawPower);
        motorB.setRunMode(Motor.RunMode.RawPower);
        motorA.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        if (RESET_ENCODERS_ON_INIT) {
            resetEncoders();
        }

        steerPID = new PIDController(STEER_KP, STEER_KI, STEER_KD);
        steerPID.setTolerance(STEER_TOLERANCE_RADIANS);
        steerPID.setMaxOutput(1.0);
    }

    public double getAngleRadians() {
        return getSteerTicks() * TICKS_TO_RADIANS_STEER;
    }

    public Rotation2d getAngle() {
        return new Rotation2d(getAngleRadians());
    }

    public double getDriveDistanceMeters() {
        return getDriveTicks() * TICKS_TO_METERS_DRIVE;
    }

    public double getDriveVelocityMetersPerSecond() {
        return getDriveTicksPerSecond() * TICKS_TO_METERS_DRIVE;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getAngle());
    }

    public void setState(SwerveModuleState desiredState) {
        SwerveModuleState optimized = Math.abs(desiredState.speedMetersPerSecond) < MODULE_OPTIMIZE_MIN_SPEED_MPS
                ? desiredState
                : SwerveModuleState.optimize(desiredState, getAngle());

        double drivePercent = clamp(
                optimized.speedMetersPerSecond / MAX_DRIVE_SPEED_MPS,
                -1.0,
                1.0
        );

        double angleError = wrapRadians(optimized.angle.getRadians() - getAngleRadians());
        double steerPercent;
        if (Math.abs(angleError) <= STEER_TOLERANCE_RADIANS) {
            steerPID.reset();
            steerPercent = 0.0;
        } else {
            steerPercent = clamp(steerPID.calculate(0.0, angleError), -1.0, 1.0);
        }

        applyDifferential(drivePercent, steerPercent);
    }

    public void stop() {
        motorA.set(0.0);
        motorB.set(0.0);
    }

    public void resetEncoders() {
        motorA.stopAndResetEncoder();
        motorB.stopAndResetEncoder();
    }

    public double getSteerTicks() {
        return (motorA.getCurrentPosition() + motorB.getCurrentPosition()) / 2.0;
    }

    public double getDriveTicks() {
        return (motorA.getCurrentPosition() - motorB.getCurrentPosition()) / 2.0;
    }

    private double getDriveTicksPerSecond() {
        return (motorA.getVelocity() - motorB.getVelocity()) / 2.0;
    }

    private void applyDifferential(double drive, double steer) {
        double scaledDrive = drive * DIFFERENTIAL_DRIVE_POWER_SCALE;
        double scaledSteer = steer * DIFFERENTIAL_STEER_POWER_SCALE;

        double powerA = scaledSteer + scaledDrive;
        double powerB = scaledSteer - scaledDrive;

        double maxPower = Math.max(Math.abs(powerA), Math.abs(powerB));
        if (maxPower > 1.0) {
            powerA /= maxPower;
            powerB /= maxPower;
        }

        motorA.set(powerA);
        motorB.set(powerB);
    }

    private double wrapRadians(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
