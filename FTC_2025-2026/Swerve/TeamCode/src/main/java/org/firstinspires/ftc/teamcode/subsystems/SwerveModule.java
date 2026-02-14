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
    private final PIDController drivePidController;  // 新增：Drive PID Controller

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private double previousAngle = 0;
    private long previousTime = System.currentTimeMillis();

    // Turning PID 監測變數
    private double targetAngle = 0;
    private double currentAngle = 0;
    private double Error = 0;
    private double Output = 0;
    private Double filteredAngleRad = null; // 轉向角度濾波緩存
    private double smoothedTargetAngle = 0; // 平滑後的目標角度
    private boolean targetIsTransitioning = false; // 目標正在過渡中

    // Drive PID 監測變數
    private double targetVelocity = 0;
    private double currentVelocity = 0;
    private double driveError = 0;
    private double driveOutput = 0;

    /**
     * SwerveModule 建構函式
     * @param hardwareMap 硬體映射物件
     * @param driveMotorName 驅動馬達名稱
     * @param turningServoName 轉向伺服機名稱
     * @param absoluteEncoderName 絕對編碼器名稱
     * @param driveMotorReversed 驅動馬達是否反向
     * @param turningMotorReversed 轉向馬達是否反向
     * @param absoluteEncoderOffset 絕對編碼器偏移量(弧度)
     * @param absoluteEncoderReversed 絕對編碼器是否反向
     */
    public SwerveModule(HardwareMap hardwareMap,
                        String driveMotorName,
                        String turningServoName,
                        String absoluteEncoderName,
                        boolean driveMotorReversed,
                        boolean turningMotorReversed,
                        double absoluteEncoderOffset,
                        boolean absoluteEncoderReversed){

        // absoluteEncoder 的參數設定
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = hardwareMap.get(AnalogInput.class, absoluteEncoderName);

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

        // 建立 PID Controller 用於驅動速度控制
        drivePidController = new PIDController(
                ModuleConstants.kPDrive,
                ModuleConstants.kIDrive,
                ModuleConstants.kDDrive);

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
        return applyAngleFilters(getAbsoluteEncoderRad());
    }

    public double getDriveVelocity() {
        // FTC 的 getVelocity() 回傳 ticks per second
        // 需要使用速度轉換因子將其轉換為 m/s
        // 等效於 driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec)
        return driveMotor.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    /**
     * 取得驅動馬達的 RPM
     * @return 馬達轉速 (rotations per minute)
     */
    public double getDriveRPM() {
        // getVelocity() 回傳 ticks per second
        // goBILDA 馬達: 28 ticks/revolution
        // RPM = (ticks/sec) / (ticks/rev) * 60
        double ticksPerSec = driveMotor.getVelocity();
        return (ticksPerSec / 28.0) * 60.0;
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

        // 如果反向，先乘以 -1
        if (absoluteEncoderReversed) {
            angle = -angle;
        }

        // 標準化角度到 [-π, π] 範圍
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;

        return angle;
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
        // 如果速度接近零，只停止驅動馬達，但繼續維持轉向角度
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            driveMotor.setPower(0);
            // 不 return，繼續執行轉向 PID 來維持角度
            // 但不更新 targetAngle，保持在上一個目標角度
            maintainTurningAngle();
            return;
        }

        // 優化模組狀態（避免超過90度旋轉）
        state = SwerveModuleState.optimize(state, getState().angle);

        // ===== Drive Motor 控制 =====
        targetVelocity = state.speedMetersPerSecond;
        currentVelocity = getDriveVelocity();

        if (TuningConfig.enableDrivePID) {
            // 使用 PID 控制驅動馬達
            drivePidController.setPID(
                    TuningConfig.driveP,
                    TuningConfig.driveI,
                    TuningConfig.driveD);

            // 計算 PID 輸出
            double pidOutput = drivePidController.calculate(currentVelocity, targetVelocity);

            // 前饋項：基於目標速度的基本功率
            double feedforward = TuningConfig.driveF * (targetVelocity / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

            // 合併 PID 和前饋
            driveOutput = (pidOutput + feedforward) * TuningConfig.driveOutputScale;

            // 計算誤差供監測
            driveError = targetVelocity - currentVelocity;

            // 限制輸出範圍
            driveOutput = Math.max(-1.0, Math.min(1.0, driveOutput));

            driveMotor.setPower(driveOutput);
        } else {
            // 簡單的開環控制（原本的方式）
            driveOutput = state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveError = 0;
            driveMotor.setPower(driveOutput);
        }

        // ===== Turning Motor 控制 =====
        // 讀取當前角度（帶跳變過濾）
        currentAngle = getTurningPosition();
        double rawTargetAngle = state.angle.getRadians();

        // 目標角度平滑化：
        // 1. 微小變化（< 死區）：忽略，避免抖動
        // 2. 大角度變化：逐漸過渡，避免突變造成震盪
        double targetDelta = normalizeAngle(rawTargetAngle - smoothedTargetAngle);
        double targetDeltaDeg = Math.abs(Math.toDegrees(targetDelta));

        if (targetDeltaDeg < TuningConfig.deadbandDeg) {
            // 微小變化：忽略，目標穩定
            targetIsTransitioning = false;
        } else if (targetDeltaDeg > TuningConfig.maxJumpDeg) {
            // 大角度變化：每循環最多移動 maxJumpDeg 度
            double maxStep = Math.toRadians(TuningConfig.maxJumpDeg);
            smoothedTargetAngle = normalizeAngle(smoothedTargetAngle + Math.copySign(maxStep, targetDelta));
            targetIsTransitioning = true; // 標記正在過渡
        } else {
            // 中等變化：直接更新
            smoothedTargetAngle = rawTargetAngle;
            targetIsTransitioning = false;
        }
        targetAngle = smoothedTargetAngle;

        Error = normalizeAngle(targetAngle - currentAngle);

        double errorDeg = Math.abs(Math.toDegrees(Error));

        // Apply live-tuned PID gains before calculating output
        turningPidController.setPID(
                TuningConfig.turningP,
                TuningConfig.turningI,
                TuningConfig.turningD);

        // 使用環繞處理後的誤差計算 PID 輸出
        this.Output = turningPidController.calculate(0, Error) * TuningConfig.turningOutputScale;

        // 當目標正在過渡時，限制最大輸出以避免過衝
        if (targetIsTransitioning) {
            double maxOut = TuningConfig.maxTransitionOutput;
            Output = Math.max(-maxOut, Math.min(maxOut, Output));
        }

        Output = applyTurningOutputConstraints(Output, errorDeg);

        // 設定轉向馬達功率
        turningMotor.setPower(Output);
    }

    public void stop() {
        driveMotor.setPower(0);
        turningMotor.setPower(0);
    }

    /**
     * 維持當前的目標轉向角度（用於速度為零時保持輪子方向）
     */
    private void maintainTurningAngle() {
        // 讀取當前角度（帶跳變過濾）
        currentAngle = getTurningPosition();
        Error = normalizeAngle(targetAngle - currentAngle);

        double errorDeg = Math.abs(Math.toDegrees(Error));

        // Apply live-tuned PID gains
        turningPidController.setPID(
                TuningConfig.turningP,
                TuningConfig.turningI,
                TuningConfig.turningD);

        // 使用環繞處理後的誤差計算 PID 輸出
        this.Output = turningPidController.calculate(0, Error) * TuningConfig.turningOutputScale;
        Output = applyTurningOutputConstraints(Output, errorDeg);

        // 設定轉向馬達功率
        turningMotor.setPower(Output);
    }

    /**
     * 直接設定轉向馬達功率 (用於 PID 調試)
     * @param power 馬達功率 [-1.0, 1.0]
     */
    public void setTurningPower(double power) {
        turningMotor.setPower(power);
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
    }

    // Telemetry helpers for dashboard/panels
    public double getTargetAngleRad() { return targetAngle; }
    public double getCurrentAngleRad() { return currentAngle; }
    public double getTurningErrorRad() { return Error; }
    public double getTurningOutput() { return Output; }

    // Drive PID telemetry helpers
    public double getTargetVelocity() { return targetVelocity; }
    public double getCurrentVelocityMps() { return currentVelocity; }
    public double getDriveError() { return driveError; }
    public double getDriveOutput() { return driveOutput; }


    /**
     * 將角度標準化到 [-π, π] 範圍
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double applyAngleFilters(double rawAngle) {
        double angle = normalizeAngle(rawAngle);
        if (filteredAngleRad != null) {
            double delta = normalizeAngle(angle - filteredAngleRad);
            double deltaDeg = Math.abs(Math.toDegrees(delta));
            double maxDeltaDeg = Math.max(0, TuningConfig.maxJumpDeg);
            if (maxDeltaDeg > 0 && deltaDeg > maxDeltaDeg) {
                double limitedDelta = Math.toRadians(maxDeltaDeg) * Math.signum(delta);
                angle = normalizeAngle(filteredAngleRad + limitedDelta);
            } else {
                angle = normalizeAngle(filteredAngleRad + delta);
            }
        }
        filteredAngleRad = angle;
        return angle;
    }

    private double applyTurningOutputConstraints(double rawOutput, double errorDeg) {
        // 死區：誤差很小直接停
        if (errorDeg < TuningConfig.deadbandDeg) {
            return 0;
        }

        double output = Math.max(-1.0, Math.min(1.0, rawOutput));

        // 最小輸出：只在誤差較大時套用，避免接近目標時震盪
        // 誤差大於 minOutputThresholdDeg 度時才強制最小輸出
        double minOut = Math.abs(TuningConfig.minOutput);
        double minOutputThreshold = TuningConfig.deadbandDeg * 3; // 誤差大於死區3倍時才套用
        if (minOut > 0 && errorDeg > minOutputThreshold && Math.abs(output) < minOut) {
            output = Math.copySign(minOut, output);
        }

        return output;
    }
}
