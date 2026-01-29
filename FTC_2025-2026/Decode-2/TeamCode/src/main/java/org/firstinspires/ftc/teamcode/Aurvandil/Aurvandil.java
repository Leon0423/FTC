package org.firstinspires.ftc.teamcode.Aurvandil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Aurvandil", group = "TeleOp")
public class Aurvandil extends LinearOpMode {

    // ===== 底盤馬達 =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== 射球機構 =====
    private DcMotorEx shooterMotor;      // goBILDA 5202-0002-0001 (28 CPR)
    private DcMotor intake1, intake2;

    // ===== 編碼器參數 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // 每圈 28 個 ticks

    // ===== 預設速度設定 =====
    private static final double LOW_RPM = 3720;   // 近距離射球速度
    private static final double HIGH_RPM = 4757.14;  // 遠距離射球速度

    // ===== RPM 微調參數 =====
    private double targetRPM = 0;                 // 當前目標 RPM（可動態調整）

    // ===== 速度容差（依模式自動切換）=====
    private static final double HIGH_VELOCITY_TOLERANCE = 40;  // 高速模式容差
    private static final double LOW_VELOCITY_TOLERANCE = 40;   // 低速模式容差

    // ===== Servo 功率設定 =====
    private static final double FEEDER_OUTTAKE_POWER = 1.0;   // 吐球功率
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
        telemetry.addLine("【Shooter 控制】");
        telemetry.addLine("  X           : 遠距離射球 (HIGH)");
        telemetry.addLine("  D-pad Left : 近距離射球 (LOW)");
        telemetry.addLine("  Right Bumper: 緊急停止");
        telemetry.addLine();
        telemetry.addLine("【送球控制】");
        telemetry.addLine("  Y (按住)   : 啟動送球 (轉速達標時)");
        telemetry.addLine();
        telemetry.addLine("【Intake 控制】");
        telemetry.addLine("  A          : 啟動吸球");
        telemetry.addLine("  B          : 停止吸球");

        waitForStart();

        // ===== 主循環 =====
        while (opModeIsActive()) {
            handleDriveControls();        // 底盤控制
            handleShooterControls();      // Shooter 啟動/停止
            handleFeederControls();       // Feeder 送球邏輯
            handleIntakeControls();       // Intake 吸球
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
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Shooter 設定（速度控制 + PIDF）
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake2.setDirection(DcMotor.Direction.REVERSE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        stopAllMotors();
    }

    // 處理底盤控制（麥克納姆輪全向移動）
    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;  // 前後
        double rotate = gamepad1.right_stick_x;   // 旋轉
        double strafe = gamepad1.left_stick_x;    // 平移
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
            intake1.setPower(0);
            intake2.setPower(0);
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
     * - Y：啟動送球（前提是速度達標）
     * - 鬆開按鈕：停止送球（可選擇吐球或靜止）
     *
     * ⚠️ 重點：容差根據 isHighVelocityMode 決定，與 RPM 微調同步
     */
    private void handleFeederControls() {
        boolean yHeld = gamepad1.y;

        double currentVelocity = shooterMotor.getVelocity();

        if (yHeld) {
            // 根據當前模式決定容差（與微調後的 RPM 同步）
            double tolerance = isHighVelocityMode ? HIGH_VELOCITY_TOLERANCE : LOW_VELOCITY_TOLERANCE;
            handleFeedLogic(currentVelocity, caculateTargetVelocity(targetRPM), tolerance);
        } else {
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
            intake2.setPower(0);
            feedEnabled = false;
            return;
        }

        boolean inRange = Math.abs(currentVelocity - targetVelocity) <= tolerance;  // ✅ 雙向容差

        // 遲滯控制
        if (!feedEnabled && inRange) {
            feedEnabled = true;  // 速度進入容差範圍，開始送球
        } else if (feedEnabled && !inRange) {
            feedEnabled = false; // 速度超出容差範圍，停止送球
        }

        intake2.setPower(feedEnabled ? FEEDER_OUTTAKE_POWER : 0.0);
    }


    /**
     * 處理 Intake 吸球控制
     * - A 按鈕：啟動吸球
     * - B 按鈕：停止吸球
     */
    private void handleIntakeControls() {
        if (gamepad1.a) {
            intake1.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intake1.setPower(0);
        }
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
        intake1.setPower(0);
        intake2.setPower(0);
    }
}