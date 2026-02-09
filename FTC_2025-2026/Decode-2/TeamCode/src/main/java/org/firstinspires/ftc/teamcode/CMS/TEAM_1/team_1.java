package org.firstinspires.ftc.teamcode.CMS.TEAM_1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "team_1_AdjustRPM", group = "TeleOp")
public class team_1 extends LinearOpMode {

    // ===== 底盤馬達 =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== 射球機構 =====
    private DcMotorEx shooterMotor;      // goBILDA 5202-0002-0001 (28 CPR)
    private CRServo feederServo;         // 連續旋轉伺服馬達

    // ===== 吸球機構 =====
    private DcMotor intakeMotor;         // 312 RPM 吸球馬達

    // ===== 編碼器參數 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // 每圈 28 個 ticks

    // ===== PIDF 係數 =====
    private static final double SHOOTER_P = 19;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 12.0334;

    // ===== 預設速度設定 =====
    private static final double LOW_RPM = 3720;   // 近距離射球速度
    private static final double HIGH_RPM = 4757.14;  // 遠距離射球速度

    // ===== RPM 微調參數 =====
    private double targetRPM = 0;                 // 當前目標 RPM（可動態調整）
    private static final double RPM_ADJUST_STEP = 50; // 每次微調步長

    // ===== 速度容差（依模式自動切換）=====
    private static final double HIGH_VELOCITY_TOLERANCE = 20;  // 高速模式容差
    private static final double LOW_VELOCITY_TOLERANCE = 20;   // 低速模式容差

    // ===== Servo 功率設定 =====
    private static final double FEEDER_OUTTAKE_POWER = 1.0;   // 吐球功率
    private static final double FEEDER_FEED_POWER = -1.0;     // 送球功率
    private static final double INTAKE_POWER = 0.5;           // 吸球功率

    // ===== 機構狀態旗標 =====
    private boolean shooterOn = false;           // Shooter 是否啟動
    private boolean feedEnabled = false;         // Feeder 是否在送球
    private boolean isHighVelocityMode = true;   // 當前速度模式（高速 true/低速 false）

    // ===== 按鍵邊緣檢測（防止連續觸發）=====
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevDpadLeft = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData("Status", "✓ 初始化完成");
        telemetry.update();

        waitForStart();

        // ===== 主循環 =====
        while (opModeIsActive()) {
            handleDriveControls();        // 底盤控制
            handleShooterControls();      // Shooter 啟動/停止
            handleFeederControls();       // Feeder 送球邏輯
            handleIntakeControls();       // Intake 吸球
            handleRPMAdjustment();        // RPM 微調
            updateTelemetry();            // 更新遙測資訊
        }

        stopAllMotors();
    }

    /**
     * 初始化所有硬體設備
     */
    private void initializeHardware() {
        // 底盤馬達初始化
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // 右側馬達反轉（統一前進方向）
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // 設定煞車模式（停止時鎖定）
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 底盤馬達不使用編碼器（直接功率控制）
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 射球與進球機構初始化
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Shooter 設定（速度控制 + PIDF）
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // Feeder 設定（連續旋轉伺服馬達）
        feederServo.setDirection(CRServo.Direction.FORWARD);

        // Intake 設定（不使用編碼器）
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stopAllMotors();
    }

    /**
     * 處理底盤控制（麥克納姆輪全向移動）
     */
    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;  // 前後
        double rotate = gamepad1.right_stick_x;   // 平移
        double strafe = gamepad1.left_stick_x;    // 旋轉
        double fr, fl, br, bl, scale;

        fr = forward - rotate - strafe;
        fl = forward + rotate + strafe;
        br = forward - rotate + strafe;
        bl = forward + rotate - strafe;

        scale = scaling_power(fr, fl, br, bl);

        frontRight.setPower(fr / scale);
        frontLeft.setPower(fl / scale);
        backRight.setPower(br / scale);
        backLeft.setPower(bl / scale);
    }

    private double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if (max <= 1) {
            max = 1;
        }
        return max;
    }

    /**
     * 處理 Shooter 啟動/停止控制
     * - X 按鈕：啟動高速模式（遠距離）
     * - D-pad Left：啟動低速模式（近距離）
     * - Right Bumper：緊急停止所有射球機構
     */
    private void handleShooterControls() {
        boolean xNow = gamepad1.x;
        boolean backNow = gamepad1.right_bumper;
        boolean dpadLeftNow = gamepad1.dpad_left;

        // X 按鈕：啟動遠距離射球
        if (xNow && !prevX) {
            shooterOn = true;
            targetRPM = HIGH_RPM;
            isHighVelocityMode = true;
        }

        // D-pad Left：啟動近距離射球
        if (dpadLeftNow && !prevDpadLeft) {
            shooterOn = true;
            targetRPM = LOW_RPM;
            isHighVelocityMode = false;
        }

        // Right Bumper：緊急停止
        if (backNow && !prevBack) {
            shooterOn = false;
            feedEnabled = false;
            targetRPM = 0;
            shooterMotor.setVelocity(0);
            feederServo.setPower(0);
        }

        // 更新按鍵狀態（用於邊緣檢測）
        prevX = xNow;
        prevBack = backNow;
        prevDpadLeft = dpadLeftNow;

        // 設定 Shooter 速度
        if (shooterOn) {
            shooterMotor.setVelocity(caculateTargetVelocity(targetRPM));
        } else {
            shooterMotor.setVelocity(0);
        }
    }

    /**
     * 處理 Feeder 送球控制
     * - Y 或 D-pad Up：啟動送球（前提是速度達標）
     * - 鬆開按鈕：停止送球（可選擇吐球或靜止）
     *
     * ⚠️ 重點：容差根據 isHighVelocityMode 決定，與 RPM 微調同步
     */
    private void handleFeederControls() {
        boolean yHeld = gamepad1.y;
        boolean dpadUpHeld = gamepad1.dpad_up;

        double currentVelocity = shooterMotor.getVelocity();

        if (yHeld || dpadUpHeld) {
            // 根據當前模式決定容差（與微調後的 RPM 同步）
            double tolerance = isHighVelocityMode ? HIGH_VELOCITY_TOLERANCE : LOW_VELOCITY_TOLERANCE;
            handleFeedLogic(currentVelocity, caculateTargetVelocity(targetRPM), tolerance);
        } else {
            // 鬆開按鈕時吐球（可改為 0.0 改成靜止）
            feederServo.setPower(FEEDER_OUTTAKE_POWER);
            feedEnabled = false;
        }
    }

    /**
     * Feeder 送球邏輯（帶遲滯控制，防止速度震盪時頻繁切換）
     * @param currentVelocity 當前實際速度（ticks/sec）
     * @param targetVelocity  目標速度（ticks/sec）
     * @param tolerance       速度容差
     */
    private void handleFeedLogic(double currentVelocity, double targetVelocity, double tolerance) {
        if (!shooterOn) {
            feederServo.setPower(0.0);
            feedEnabled = false;
            return;
        }

        // 遲滯控制：防止速度在臨界點震盪
        if (!feedEnabled && currentVelocity >= targetVelocity) {
            feedEnabled = true;  // 速度達標，開始送球
        } else if (feedEnabled && currentVelocity <= targetVelocity - tolerance) {
            feedEnabled = false; // 速度掉太多，停止送球
        }

        feederServo.setPower(feedEnabled ? FEEDER_FEED_POWER : 0.0);
    }

    /**
     * 處理 Intake 吸球控制
     * - A 按鈕：啟動吸球
     * - B 按鈕：停止吸球
     */
    private void handleIntakeControls() {
        if (gamepad1.a) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intakeMotor.setPower(0);
        }
    }

    /**
     * 處理 RPM 微調
     * - D-pad Right：增加 50 RPM
     * - D-pad Down：減少 50 RPM
     * - 限制範圍：0 ~ 5800 RPM
     */
    private void handleRPMAdjustment() {
        if (gamepad1.dpadRightWasPressed()) {
            targetRPM += RPM_ADJUST_STEP;
        } else if (gamepad1.dpadDownWasPressed()) {
            targetRPM -= RPM_ADJUST_STEP;
        }

        // 限制 RPM 範圍
        targetRPM = Math.max(0, Math.min(targetRPM, 5800));
    }

    /**
     * 計算當前實際 RPM
     */
    private double calculateRPM() {
        if (!shooterOn) return 0.0;

        double ticksPerSecond = shooterMotor.getVelocity();
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    /**
     * 將 RPM 轉換為 ticks per second（供馬達速度控制使用）
     */
    private double caculateTargetVelocity(double RPM) {
        return RPM * (SHOOTER_TICKS_PER_REV / 60.0);
    }

    /**
     * 更新遙測資訊（優化排版）
     */
    private void updateTelemetry() {
        double rpm = calculateRPM();
        double error = rpm - targetRPM;

        telemetry.addData("🎯 Shooter", shooterOn ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("⚡ 模式", isHighVelocityMode ? "遠距離 (HIGH)" : "近距離 (LOW)");
        telemetry.addData("📦 送球狀態", feedEnabled ? "✓ 送球中" : "✗ 待命");
        telemetry.addLine();
        telemetry.addData("目標 RPM", String.format("%.0f", targetRPM));
        telemetry.addData("實際 RPM", String.format("%.0f", rpm));
        telemetry.addData("Error", String.format("%+.1f RPM", error));
        telemetry.addData("轉速達標", Math.abs(error) <= (isHighVelocityMode ? HIGH_VELOCITY_TOLERANCE : LOW_VELOCITY_TOLERANCE) ? "YES" : "NO");
        telemetry.addLine();
        telemetry.addData("➕ RPM 增加", gamepad1.dpad_right ? "按住" : "-");
        telemetry.addData("➖ RPM 減少", gamepad1.dpad_down ? "按住" : "-");

        telemetry.update();
    }

    /**
     * 停止所有馬達
     */
    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        shooterMotor.setVelocity(0);
        feederServo.setPower(0);
        intakeMotor.setPower(0);
    }
}