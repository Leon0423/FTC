package org.firstinspires.ftc.teamcode.subsystems;

import android.content.Context;
import android.content.SharedPreferences;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.ModuleConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Tuning.TuningConfig;

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

    // Drive PID 監測變數
    private double targetVelocity = 0;
    private double currentVelocity = 0;
    private double driveError = 0;
    private double driveOutput = 0;

    private double accumulatedAngle = 0;      // 累積的輪子角度（弧度）
    private double lastRawServoRad = 0;       // 上一次伺服原始角度
    private boolean turningInitialized = false;

    private double cachedTurningPosition = 0;
    private long lastUpdateTime = -1;

    private final double powerScale;

    private double filteredVelocity = 0;
    private static final double VELOCITY_FILTER_ALPHA = 0.2;

    private static final String MODULE_PREFS_NAME = "SwerveModulePersistentAngles";
    private static final String OFFSET_PREFS_NAME = "SwerveOffsetPrefs";
    private static final int SAVE_EVERY_N_UPDATES = 25;

    private final SharedPreferences modulePrefs;
    private final SharedPreferences offsetPrefs;
    private final String prefAccumulatedAngleKey;
    private final String prefLastRawKey;
    private final String prefOffsetKey;
    private boolean enableSaving = true;
    private int saveCounter = 0;

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
                        boolean absoluteEncoderReversed,
                        double powerScale){

        this.powerScale = powerScale;
        this.prefAccumulatedAngleKey = "accum_angle_" + turningServoName;
        this.prefLastRawKey = "last_raw_" + turningServoName;
        this.prefOffsetKey = "offset_" + turningServoName;
        modulePrefs = AppUtil.getDefContext().getSharedPreferences(MODULE_PREFS_NAME, Context.MODE_PRIVATE);
        offsetPrefs = AppUtil.getDefContext().getSharedPreferences(OFFSET_PREFS_NAME, Context.MODE_PRIVATE);

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

    // ===== 修改 getTurningPosition()，每次更新後儲存 =====
    public double getTurningPosition() {
        if (!turningInitialized) return getAbsoluteEncoderRad();

        long now = System.currentTimeMillis();

        // 同一毫秒內直接回傳快取值，不重複累積 delta
        if (now == lastUpdateTime) return cachedTurningPosition;
        lastUpdateTime = now;

        double currentRaw = getRawServoRad();
        double delta = currentRaw - lastRawServoRad;
        if (delta >  Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;

        accumulatedAngle += delta * ModuleConstants.kTurningMotorGearRatio;
        lastRawServoRad = currentRaw;

        if (enableSaving) {
            saveCounter++;
            if (saveCounter >= SAVE_EVERY_N_UPDATES) {
                saveTrackingState();
                saveCounter = 0;
            }
        }

        double normalized = accumulatedAngle;
        while (normalized >  Math.PI) normalized -= 2 * Math.PI;
        while (normalized < -Math.PI) normalized += 2 * Math.PI;

        cachedTurningPosition = normalized;
        return cachedTurningPosition;
    }

    public void clearSavedAngle() {
        modulePrefs.edit()
                .remove(prefAccumulatedAngleKey)
                .remove(prefLastRawKey)
                .apply();
    }

    // Legacy API: kept for call-site compatibility after removing persistent storage.
    public void clearSavedOffset() {
        offsetPrefs.edit().remove(prefOffsetKey).apply();
    }

    public double getActiveOffsetRad() {
        return loadOffsetRad();
    }

    public double getRawServoRadians() {
        return getRawServoRad();
    }

    // 連續角度（不包到 [-pi, pi]），可用於觀察跨圈累積效果。
    public double getAccumulatedWheelAngleRad() {
        return accumulatedAngle;
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

    private double getRawServoRad() {
        return (absoluteEncoder.getVoltage() / absoluteEncoder.getMaxVoltage()) * 2.0 * Math.PI;
    }

    // ===== initTurningTracking()：以絕對編碼器直接定義當前角度，避免 offset/gear 比例反覆疊加 =====
    public void initTurningTracking() {
        double currentRaw = getRawServoRad();
        double initAngle;

        if (enableSaving && modulePrefs.contains(prefAccumulatedAngleKey) && modulePrefs.contains(prefLastRawKey)) {
            // 從上次追蹤狀態續接，並先做一次跨 0/360 的增量補償。
            double savedAccumulated = modulePrefs.getFloat(prefAccumulatedAngleKey, 0f);
            double savedRaw = modulePrefs.getFloat(prefLastRawKey, (float) currentRaw);
            double delta = currentRaw - savedRaw;
            if (delta > Math.PI) delta -= 2 * Math.PI;
            if (delta < -Math.PI) delta += 2 * Math.PI;
            initAngle = savedAccumulated + delta * ModuleConstants.kTurningMotorGearRatio;
        } else {
            // 無持久化資料時，以絕對編碼器 + offset 建立初始角度。
            initAngle = currentRaw * ModuleConstants.kTurningMotorGearRatio - loadOffsetRad();
        }

        lastRawServoRad = currentRaw;
        accumulatedAngle = initAngle;

        double wrapped = initAngle;
        while (wrapped > Math.PI) wrapped -= 2 * Math.PI;
        while (wrapped < -Math.PI) wrapped += 2 * Math.PI;

        cachedTurningPosition = wrapped;
        targetAngle = wrapped;   // PID 目標對齊實際位置，不會開機亂轉

        turningInitialized = true;

        if (enableSaving) {
            saveTrackingState();
            saveCounter = 0;
        }
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / absoluteEncoder.getMaxVoltage();
        angle *= 2.0 * Math.PI;
        angle -= loadOffsetRad();
        if (absoluteEncoderReversed) angle = -angle;
        while (angle >  Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public double getAbsoluteEncoderVoltage() {
        return absoluteEncoder.getVoltage();
    }

    public void resetEncoders() {
        resetEncoders(false);
    }

    public void resetEncoders(boolean clearSavedAngle) {
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (clearSavedAngle) {
            clearSavedAngle();
        }
        resetTurningTracking(clearSavedAngle);
    }

    /**
     * Reset turning tracking state so delta accumulation restarts from the current absolute encoder angle.
     */
    public void resetTurningTracking(boolean clearSavedAngle) {
        turningInitialized = false;
        accumulatedAngle = 0;
        cachedTurningPosition = 0;
        lastUpdateTime = -1;

        initTurningTracking();

        previousAngle = cachedTurningPosition;
        previousTime = System.currentTimeMillis();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        // 如果速度接近零，只停止驅動馬達，但繼續維持轉向角度
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            driveMotor.setPower(0);
            // 零速時也要更新目標角，避免沿用舊 targetAngle 導致開場亂偏。
            targetAngle = state.angle.getRadians();
            maintainTurningAngle();
            return;
        }

        // 優化模組狀態（避免超過90度旋轉）
        state = SwerveModuleState.optimize(state, getState().angle);

        // ===== Drive Motor 控制 =====
        targetVelocity = state.speedMetersPerSecond;
        currentVelocity = getDriveVelocity();

        if (TuningConfig.enableDrivePID()) {
            // 使用 PID 控制驅動馬達
            drivePidController.setPID(
                    TuningConfig.driveP(),
                    TuningConfig.driveI(),
                    TuningConfig.driveD());

            // 計算 PID 輸出
            double pidOutput = drivePidController.calculate(currentVelocity, targetVelocity);

            // 前饋項：基於目標速度的基本功率
            double feedforward = TuningConfig.driveF() * (targetVelocity / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

            // 合併 PID 和前饋
            driveOutput = (pidOutput + feedforward) * TuningConfig.driveOutputScale();

            // 計算誤差供監測
            driveError = targetVelocity - currentVelocity;

            // 限制輸出範圍
            driveOutput = Math.max(-1.0, Math.min(1.0, driveOutput));

            driveMotor.setPower(driveOutput);
        } else {
            // 簡單的開環控制（原本的方式）
            driveOutput = state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * powerScale;
            driveError = 0;
            driveMotor.setPower(driveOutput);
        }

        // ===== Turning Motor 控制 (與 TurningPIDTuner 相同) =====
        currentAngle = getTurningPosition();
        targetAngle = state.angle.getRadians();

        Error = normalizeAngle(targetAngle - currentAngle);
        Output = computeTurningOutput(Error);
        turningMotor.setPower(Output);
    }

    public void stop() {
        driveMotor.setPower(0);
        turningMotor.setPower(0);

        if (enableSaving) {
            saveTrackingState();
        }
    }

    /**
     * 設定模組目標狀態（無 Drive PID，直接功率控制）
     * 轉向仍使用 PID，但驅動馬達直接用功率
     * @param state 目標狀態
     */
    public void setDesiredStateNoPID(SwerveModuleState state) {
        // 如果速度接近零，停止驅動但維持轉向
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            driveMotor.setPower(0);
            targetAngle = state.angle.getRadians();
            maintainTurningAngle();
            return;
        }

        // 優化模組狀態（避免超過90度旋轉）
        state = SwerveModuleState.optimize(state, getState().angle);

        // ===== Drive Motor 控制（無 PID，直接功率）=====
        double drivePower = state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * powerScale;
        driveMotor.setPower(drivePower);

        // ===== Turning Motor 控制（仍使用 PID）=====
        currentAngle = getTurningPosition();
        targetAngle = state.angle.getRadians();

        Error = normalizeAngle(targetAngle - currentAngle);
        Output = computeTurningOutput(Error);
        turningMotor.setPower(Output);
    }

    /**
     * 維持當前的目標轉向角度（用於速度為零時保持輪子方向）
     */
    private void maintainTurningAngle() {
        currentAngle = getTurningPosition();

        Error = normalizeAngle(targetAngle - currentAngle);
        Output = computeTurningOutput(Error);
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
     * 直接設定驅動馬達功率（繞過 PID，用於手動控制）
     * @param power 馬達功率 [-1.0, 1.0]
     */
    public void setDriveMotorPowerDirect(double power) {
        driveMotor.setPower(power);
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

    public void disableSaving() { enableSaving = false; }

    public void enableSaving()  { enableSaving = true; }

    public void alignTurningOnly(double targetRad) {
        currentAngle = getTurningPosition();
        targetAngle = targetRad;

        Error = normalizeAngle(targetAngle - currentAngle);
        Output = computeTurningOutput(Error);
        turningMotor.setPower(Output);
        // ★ 完全不碰 driveMotor
    }

    public void setDriveDirection(boolean reversed) {
        driveMotor.setDirection(reversed ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void resetDriveEncoder() {
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getDriveEncoderPosition() {
        return driveMotor.getCurrentPosition();
    }

    // 為轉向 PID 計算輸出（移除 minOutput 以避免震盪）
    private double computeTurningOutput(double errorRad) {
        double errorDeg = Math.abs(Math.toDegrees(errorRad));

        turningPidController.setPID(
                TuningConfig.turningP(),
                TuningConfig.turningI(),
                TuningConfig.turningD());

        if (errorDeg < TuningConfig.deadbandDeg()) return 0;

        double output = turningPidController.calculate(0, errorRad) * TuningConfig.turningOutputScale();
        return Math.max(-1.0, Math.min(1.0, output));
    }

    public double getDriveVelocity() {
        double raw = driveMotor.getVelocity() * ModuleConstants.kDriveEncoderRot2Meter;
        filteredVelocity = VELOCITY_FILTER_ALPHA * raw + (1 - VELOCITY_FILTER_ALPHA) * filteredVelocity;
        return filteredVelocity;
    }

    public double getRawDriveVelocity() {
        return driveMotor.getVelocity() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    private void saveTrackingState() {
        modulePrefs.edit()
                .putFloat(prefAccumulatedAngleKey, (float) accumulatedAngle)
                .putFloat(prefLastRawKey, (float) lastRawServoRad)
                .apply();
    }

    private double loadOffsetRad() {
        if (offsetPrefs.contains(prefOffsetKey)) {
            return offsetPrefs.getFloat(prefOffsetKey, (float) absoluteEncoderOffsetRad);
        }
        return absoluteEncoderOffsetRad;
    }
}
