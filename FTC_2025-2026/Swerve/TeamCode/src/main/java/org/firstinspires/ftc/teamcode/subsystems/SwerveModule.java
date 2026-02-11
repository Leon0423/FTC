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
import org.firstinspires.ftc.teamcode.TuningConfig;

public class SwerveModule {

    private final DcMotorEx driveMotor; // 內建Encoder
    private final CRServo turningMotor;

    private final AnalogInput absoluteEncoder; // turningMotor的Encoder

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private double previousAngle = 0;
    private long previousTime = System.currentTimeMillis();

    // PID 監測變數
    private double targetAngle = 0;
    private double currentAngle = 0;
    private double Error = 0;
    private double Output = 0;

    public SwerveModule(HardwareMap hardwareMap,
                        String driveMotorName,
                        String turningServoName,
                        boolean driveMotorReversed,
                        boolean turningMotorReversed,
                        int absoluteEncoderId,
                        double absoluteEncoderOffset,
                        boolean absoluteEncoderReversed){

        // absoluteEncoder 的參數設定
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "absoluteEncoder" + absoluteEncoderId);

        // driveMotor 的參數設定
        driveMotor = hardwareMap.get(DcMotorEx.class, driveMotorName);
        driveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // driveMotor 的 Encoder 設定
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // turningMotor 的參數設定
        turningMotor = hardwareMap.get(CRServo.class, turningServoName);


        // 設定馬達正反轉
        driveMotor.setDirection(driveMotorReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turningMotor.setDirection(turningMotorReversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        // 建立 PID Controller 用於旋轉控制
        turningPidController = new PIDController(
                ModuleConstants.kPTurning,
                ModuleConstants.kITurning,
                ModuleConstants.kDTurning);

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
        // 使用差分計算角速度
        long currentTime = System.currentTimeMillis();
        double currentAngle = getTurningPosition();
        double dt = (currentTime - previousTime) / 1000.0;  // 轉換為秒

        if (dt < 0.001) return 0;

        double velocity = (currentAngle - previousAngle) / dt;
        previousAngle = currentAngle;
        previousTime = currentTime;

        return velocity;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / absoluteEncoder.getMaxVoltage();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;

        // 新增：標準化角度到 [-π, π] 範圍
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsoluteEncoderVoltage() {
        return absoluteEncoder.getVoltage();
    }

    public void resetEncoders() {
        // 重置驅動馬達編碼器
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 重置速度計算變數
        previousAngle = getTurningPosition();
        previousTime = System.currentTimeMillis();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        // 如果速度接近零，停止模組
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // 優化模組狀態（避免超過90度旋轉）
        state = SwerveModuleState.optimize(state, getState().angle);

        // 設定驅動馬達速度
        driveMotor.setPower(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        // 計算角度誤差（已經有正確的環繞處理）
        currentAngle = getTurningPosition();
        targetAngle = state.angle.getRadians();
        Error = targetAngle - currentAngle;

        // 將誤差環繞到 [-π, π]
        while (Error > Math.PI) Error -= 2 * Math.PI;
        while (Error < -Math.PI) Error += 2 * Math.PI;

        // Apply live-tuned PID gains before calculating output
        turningPidController.setPID(
                TuningConfig.turningP,
                TuningConfig.turningI,
                TuningConfig.turningD);

        // Use PID controller to compute output and scale it
        this.Output = turningPidController.calculate(currentAngle, targetAngle);
        Output *= TuningConfig.turningOutputScale;

        // 新增：限制輸出範圍到 [-1.0, 1.0]
        Output = Math.max(-1.0, Math.min(1.0, Output));

        // 新增：小誤差時降低輸出，避免抖動
        if (Math.abs(Error) < 0.05) {  // 約 3 度
            Output *= 0.5;  // 降低功率
        }

        // 設定轉向馬達功率
        turningMotor.setPower(Output);
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

    // Telemetry helpers for dashboard/panels
    public double getTargetAngleRad() { return targetAngle; }
    public double getCurrentAngleRad() { return currentAngle; }
    public double getTurningErrorRad() { return Error; }
    public double getTurningOutput() { return Output; }
}
