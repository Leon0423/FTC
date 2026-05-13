package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.OIConstants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier  xSpdFunction, ySpdFunction, turningSpdFunction;
    private final BooleanSupplier fieldOrientedFunction;
    private final BooleanSupplier yLockModeFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                             DoubleSupplier xSpdFunction,
                             DoubleSupplier ySpdFunction,
                             DoubleSupplier turningSpdFunction,
                             BooleanSupplier fieldOrientedFunction,
                             BooleanSupplier yLockModeFunction) {
        this.swerveSubsystem       = swerveSubsystem;
        this.xSpdFunction          = xSpdFunction;
        this.ySpdFunction          = ySpdFunction;
        this.turningSpdFunction    = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.yLockModeFunction     = yLockModeFunction;
        this.xLimiter       = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter       = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed       = xSpdFunction.getAsDouble();
        double ySpeed       = ySpdFunction.getAsDouble();
        double turningSpeed = turningSpdFunction.getAsDouble();

        xSpeed       = Math.abs(xSpeed)       > OIConstants.kDeadband ? xSpeed       : 0.0;
        ySpeed       = Math.abs(ySpeed)       > OIConstants.kDeadband ? ySpeed       : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        boolean sticksIdle = (xSpeed == 0 && ySpeed == 0 && turningSpeed == 0);

        if (sticksIdle) {
            if (yLockModeFunction.getAsBoolean()) {
                swerveSubsystem.setXLockPattern();
            } else {
                xLimiter.reset();
                yLimiter.reset();
                turningLimiter.reset();
                swerveSubsystem.stopModules();
            }
            return;
        }

        xSpeed       = xLimiter.calculate(xSpeed)       * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed       = yLimiter.calculate(ySpeed)        * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                       * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.getAsBoolean()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ── SlewRateLimiter (WPILib equivalent) ──────────────────────────────

    private static class SlewRateLimiter {
        private final double rateLimit;
        private double prevVal  = 0;
        private long   prevTime = -1;

        public SlewRateLimiter(double rateLimit) {
            this.rateLimit = rateLimit;
        }

        public double calculate(double input) {
            long now = System.nanoTime();
            if (prevTime < 0) {
                prevTime = now;
                prevVal  = input;
                return input;
            }
            double dt        = (now - prevTime) / 1e9;
            prevTime         = now;
            double maxChange = rateLimit * dt;
            double change    = Math.max(-maxChange, Math.min(maxChange, input - prevVal));
            prevVal         += change;
            return prevVal;
        }

        public void reset() {
            prevVal  = 0;
            prevTime = -1;
        }
    }
}
