package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ModuleConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

public class SwerveModule {

    private final DcMotorEx driveMotor;
    // 內建Encoder

    private final CRServo turningMotor;
    private final AnalogInput absoluteEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(HardwareMap hardwareMap, int driveMotorId, int turningServoID, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "absoluteEncoder" + absoluteEncoderId);

        driveMotor = hardwareMap.get(DcMotorEx.class, "driveMotor" + driveMotorId);
        driveMotor.setDirection(driveMotorReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        driveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turningMotor = hardwareMap.get(CRServo.class, "turningServo" + turningServoID);
        if (turningMotorReversed) {
            turningMotor.setDirection(CRServo.Direction.REVERSE);
        }

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getCurrentPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getDriveVelocity() {
        // FTC 的 getVelocity() 回傳 ticks per second
        return driveMotor.getVelocity() * ModuleConstants.kDriveEncoderRot2Meter /
                (2 * Math.PI); // 轉換為 m/s
    }

    public double getTurningPosition() {
        return getAbsoluteEncoderRad();
    }

    public double getTurningVelocity() {
        // AnalogInput 沒有速度方法，需要自行計算或使用濾波器
        return 0; // 暫時回傳 0，或實作差分計算
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / absoluteEncoder.getMaxVoltage();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 如果有裝Bore Encoder再刪註解
        // turningMotor.set(getAbsoluteEncoderRad());
        turningMotor.
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.setPower(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        double currentAngle = getAbsoluteEncoderRad();
        double error = state.angle.getRadians() - currentAngle;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        double output = turningPidController.calculate(0, error);
        double servoPosition = 0.5 + (output / (2 * Math.PI));
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
        turningMotor.setPosition(servoPosition);

    }

    public void stop() {
        driveMotor.setPower(0);
        turningMotor.setPosition();
    }

}
