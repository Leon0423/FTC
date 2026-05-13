package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class SwerveModule {

    private final String name;
    private final DcMotorEx driveMotor;
    private final CRServo turnServo;
    private final AnalogInput analogInput;
    private final boolean analogReversed;
    private final double forwardOffsetDeg;

    private final SimplePID steerPID;

    private double targetWheelAngleDeg = 0.0;
    private double targetDrivePower = 0.0;
    private double lastSteerPower = 0.0;

    public SwerveModule(
            HardwareMap hardwareMap,
            String name,
            String driveName,
            boolean driveReversed,
            String turnName,
            boolean turnReversed,
            String analogName,
            boolean analogReversed,
            double forwardOffsetDeg
    ) {
        this.name = name;
        this.analogReversed = analogReversed;
        this.forwardOffsetDeg = forwardOffsetDeg;

        driveMotor = hardwareMap.get(DcMotorEx.class, driveName);
        turnServo = hardwareMap.get(CRServo.class, turnName);
        analogInput = hardwareMap.get(AnalogInput.class, analogName);

        driveMotor.setDirection(driveReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turnServo.setDirection(turnReversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        steerPID = new SimplePID(
                SwerveConstants.STEER_kP,
                SwerveConstants.STEER_kI,
                SwerveConstants.STEER_kD,
                SwerveConstants.STEER_kF
        );
    }

    public void refreshPID() {
        steerPID.setCoefficients(
                SwerveConstants.STEER_kP,
                SwerveConstants.STEER_kI,
                SwerveConstants.STEER_kD,
                SwerveConstants.STEER_kF
        );
    }

    public double getAnalogVoltage() {
        return analogInput.getVoltage();
    }

    public double getAnalogMaxVoltage() {
        return analogInput.getMaxVoltage();
    }

    public double getRawAnalogAngleDeg() {
        double max = getAnalogMaxVoltage();
        if (max <= 0.0) return 0.0;

        double normalized = getAnalogVoltage() / max;
        double deg = normalized * SwerveConstants.ANALOG_FULL_SCALE_DEG;

        if (analogReversed) {
            deg = SwerveConstants.ANALOG_FULL_SCALE_DEG - deg;
        }

        return deg;
    }

    public double getWheelAngleDeg() {
        return wrapDegrees(getRawAnalogAngleDeg() - forwardOffsetDeg);
    }

    public void setDesiredState(double speedPower, double desiredWheelAngleDeg) {
        double currentAngle = getWheelAngleDeg();
        double delta = angleDiffDeg(desiredWheelAngleDeg, currentAngle);

        if (Math.abs(delta) > 90.0) {
            desiredWheelAngleDeg = wrapDegrees(desiredWheelAngleDeg + 180.0);
            speedPower = -speedPower;
        }

        targetWheelAngleDeg = wrapDegrees(desiredWheelAngleDeg);
        targetDrivePower = Range.clip(speedPower, -1.0, 1.0);
    }

    public void pointForward() {
        setDesiredState(0.0, 0.0);
    }

    public void update() {
        refreshPID();

        double currentWheelDeg = getWheelAngleDeg();
        double errorDeg = angleDiffDeg(targetWheelAngleDeg, currentWheelDeg);

        double steerPower = steerPID.updateClamped(
                errorDeg,
                -SwerveConstants.STEER_MAX_POWER,
                SwerveConstants.STEER_MAX_POWER
        );

        if (Math.abs(errorDeg) < SwerveConstants.STEER_TOLERANCE_DEG) {
            steerPower = 0.0;
        }

        lastSteerPower = steerPower;
        turnServo.setPower(steerPower);

        double driveScale = 1.0;
        if (Math.abs(errorDeg) > 40.0) {
            driveScale = SwerveConstants.DRIVE_SCALE_IF_ERROR_GT_40;
        } else if (Math.abs(errorDeg) > 20.0) {
            driveScale = SwerveConstants.DRIVE_SCALE_IF_ERROR_GT_20;
        }

        driveMotor.setPower(targetDrivePower * driveScale);
    }

    public void stop() {
        driveMotor.setPower(0.0);
        turnServo.setPower(0.0);
    }

    public String getName() {
        return name;
    }

    public double getTargetWheelAngleDeg() {
        return targetWheelAngleDeg;
    }

    public double getTargetDrivePower() {
        return targetDrivePower;
    }

    public double getLastSteerPower() {
        return lastSteerPower;
    }

    public double getAngleErrorDeg() {
        return angleDiffDeg(targetWheelAngleDeg, getWheelAngleDeg());
    }

    public double getDriveMotorVelocityTicks() {
        return driveMotor.getVelocity();
    }

    public double getDriveMotorRPM() {
        return (getDriveMotorVelocityTicks() / SwerveConstants.DRIVE_MOTOR_TICKS_PER_REV) * 60.0;
    }

    public double getWheelRPM() {
        return getDriveMotorRPM() / SwerveConstants.DRIVE_GEAR_RATIO;
    }

    public double getWheelLinearSpeedMps() {
        double circumference = Math.PI * SwerveConstants.WHEEL_DIAMETER_M;
        return getWheelRPM() * circumference / 60.0;
    }

    public static double wrapDegrees(double deg) {
        while (deg >= 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    public static double angleDiffDeg(double target, double current) {
        return wrapDegrees(target - current);
    }
}