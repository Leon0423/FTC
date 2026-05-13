package org.firstinspires.ftc.teamcode.swerve;

public class SwerveTuningData {
    public double maxTranslation;
    public double maxRotation;
    public double steerKP;
    public double steerKD;
    public double steerMaxPower;
    public double steerToleranceDeg;

    public SwerveTuningData() {
        maxTranslation = SwerveConstants.MAX_TRANSLATION;
        maxRotation = SwerveConstants.MAX_ROTATION;
        steerKP = SwerveConstants.STEER_kP;
        steerKD = SwerveConstants.STEER_kD;
        steerMaxPower = SwerveConstants.STEER_MAX_POWER;
        steerToleranceDeg = SwerveConstants.STEER_TOLERANCE_DEG;
    }
}