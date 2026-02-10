package org.firstinspires.ftc.teamcode.Swerve.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Swerve.Constants.ModuleConstants;
import org.firstinspires.ftc.teamcode.Swerve.Constants.DriveConstants;

public class SwerveModule {

    private final DcMotorEx driveMotor; // 內建Encoder
    private final CRServo turningMotor;

    private final AnalogInput absoluteEncoder; // turningMotor的Encoder

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningServoId,
                        boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset,
                        boolean absoluteEncoderReversed){

        // absoluteEncoder 的參數設定
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "absoluteEncoder" + absoluteEncoderId);

        // driveMotor 的參數設定
        driveMotor = hardwareMap.get(DcMotorEx.class, "driveMotor" + driveMotorId);
        driveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // driveMotor 的 Encoder 設定
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // turningMotor 的參數設定
        turningMotor = hardwareMap.get(CRServo.class, "turningServo" + turningServoId);


        // 設定馬達正反轉
        driveMotor.setDirection(driveMotorReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turningMotor.setDirection(turningMotorReversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        // 建立 PID Controller 用於旋轉控制
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        // 注意：FTCLib 的 PIDController 沒有 enableContinuousInput 方法
        // 角度的連續性處理將在 setDesiredState 方法中手動處理

        configDriveMotor();
        configTurningMotor();

        resetEncoders();
    }

    public double getDrivePosition() {
        // FTC 中直接在讀取時應用轉換因子
        // 等效於 driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        return driveMotor.getCurrentPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        // 等效於 turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        return getAbsoluteEncoderRad();
    }

    public double getDriveVelocity() {
        // FTC 的 getVelocity() 回傳 ticks per second
        // 需要使用速度轉換因子將其轉換為 m/s
        // 等效於 driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec)
        return driveMotor.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        // AnalogInput 沒有速度方法，需要自行計算或使用濾波器
        // 等效於 turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)
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
        absoluteEncoder.resetDeviceConfigurationForOpMode();

        // 如果有裝Bore Encoder再刪註解
        // turningMotor.set(getAbsoluteEncoderRad());
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

        // Calculate angle error with proper wrapping
        double currentAngle = getTurningPosition();
        double targetAngle = state.angle.getRadians();
        double error = targetAngle - currentAngle;

        // Wrap angle error to [-π, π]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;


        // Use PID controller with proper setpoint and measurement
        double output = turningPidController.calculate(currentAngle, targetAngle);

        // Set the turning motor power directly
        turningMotor.setPower(output);

    }

    public void stop() {
        driveMotor.setPower(0);
        turningMotor.setPower(0);
    }

    /**
     * 配置驅動馬達參數
     * 在 FTC 中，encoder 轉換因子在讀取時應用，而非設定時
     * 等效於以下 FRC 程式碼：
     * driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
     * driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
     */
    private void configDriveMotor() {
        // 設定馬達為 BRAKE 模式
        driveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // 可以在這裡設定其他參數，例如：
        // - 電流限制（如果使用支援的馬達控制器）
        // - 加速度曲線
        // - PID 係數（如果使用 RUN_TO_POSITION 或 RUN_USING_ENCODER）
    }

    /**
     * 配置轉向馬達/伺服機參數
     * 等效於以下 FRC 程式碼：
     * turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
     * turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
     */
    private void configTurningMotor() {
        // CRServo 的配置選項較少
        // 可以在這裡設定其他參數，例如：
        // - PWM 範圍（如果需要）
        // - 速度限制

        // 轉向編碼器的轉換因子已在 getAbsoluteEncoderRad() 方法中處理
    }

}
