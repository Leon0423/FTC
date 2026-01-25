package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * 🎯 Team 1 最終版 TeleOp 程式
 *
 * 功能：
 * - 全向輪底盤控制
 * - 雙模式 Shooter (遠/近距離)
 * - 自動送球系統 (速度檢測 + 防卡球)
 * - Intake 吸球機構
 */
@TeleOp(name = "team_1_final", group = "TeleOp")
public class team_1_test extends LinearOpMode {

    // ===== 硬體宣告 =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;  // 底盤馬達
    private DcMotorEx shooterMotor;      // Shooter 馬達 (goBILDA 5202)
    private CRServo feederServo;         // Feeder 連續伺服馬達
    private DcMotor intakeMotor;         // Intake 馬達 (312 RPM)

    // ===== Shooter 參數 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0;  // 編碼器解析度
    private static final double SHOOTER_P = 230.0;             // PIDF 比例增益
    private static final double SHOOTER_I = 0.0;               // PIDF 積分增益
    private static final double SHOOTER_D = 0.0;               // PIDF 微分增益
    private static final double SHOOTER_F = 11.5;              // PIDF 前饋增益

    // ===== 射球速度設定 (RPM) =====
    private static final double LOW_RPM = 2000;   // 近距離目標轉速
    private static final double HIGH_RPM = 4500;  // 遠距離目標轉速

    // ===== 速度容差 (防止速度震盪) =====
    private static final double HIGH_VELOCITY_TOLERANCE = 20;  // 遠距離容差 (ticks/s)
    private static final double LOW_VELOCITY_TOLERANCE = 20;   // 近距離容差 (ticks/s)

    // ===== 功率設定 =====
    private static final double FEEDER_OUTTAKE_POWER = 1.0;   // 吐球功率 (防卡球)
    private static final double FEEDER_FEED_POWER = -1.0;     // 送球功率
    private static final double INTAKE_POWER = 0.8;           // 吸球功率

    // ===== 狀態變數 =====
    private boolean shooterOn = false;           // Shooter 是否啟動
    private boolean feedEnabled = false;         // 是否允許送球
    private boolean isHighVelocityMode = true;   // 當前速度模式

    // ===== 按鍵邊緣檢測 (防止重複觸發) =====
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevDpadLeft = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData("🤖 狀態", "已初始化");
        telemetry.addData("⚡", "等待開始...");
        telemetry.update();

        waitForStart();

        // ===== 主控制迴圈 =====
        while (opModeIsActive()) {
            handleDriveControls();     // 處理底盤移動
            handleShooterControls();   // 處理 Shooter 啟動
            handleFeederControls();    // 處理自動送球
            handleIntakeControls();    // 處理 Intake 吸球
            updateTelemetry();         // 更新螢幕顯示
        }

        stopAllMotors();  // 程式結束時停止所有馬達
    }

    /**
     * 🔧 初始化所有硬體
     */
    private void initializeHardware() {
        // --- 底盤馬達配置 ---
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // 左側馬達反轉 (確保正確的移動方向)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // 設定零功率煞車模式 (停止時鎖死)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 底盤不使用編碼器 (更快的響應)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Shooter 與 Feeder 配置 ---
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Shooter 編碼器重置並啟用速度控制
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // 停止時自由滑行

        // 設定 PIDF 控制參數 (用於精確速度控制)
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // Intake 設定
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stopAllMotors();
    }

    /**
     * 🎮 處理底盤移動控制
     * - 左搖桿 Y：前後移動
     * - 左搖桿 X：左右平移
     * - 右搖桿 X：旋轉
     */
    private void handleDriveControls() {
        double y = gamepad1.left_stick_y;   // 前後
        double rx = gamepad1.left_stick_x;  // 平移
        double x = gamepad1.right_stick_x;  // 旋轉

        // 麥克納姆輪運動學公式
        frontLeft.setPower(-y + x + rx);
        frontRight.setPower(-y - x - rx);
        backLeft.setPower(y - x + rx);
        backRight.setPower(y + x - rx);
    }

    /**
     * 🎯 處理 Shooter 啟動控制
     * - X 按鈕：啟動遠距離模式 (4500 RPM)
     * - D-pad Left：啟動近距離模式 (2000 RPM)
     * - Right Bumper：緊急停止
     */
    private void handleShooterControls() {
        boolean xNow = gamepad1.x;
        boolean backNow = gamepad1.right_bumper;
        boolean dpadLeftNow = gamepad1.dpad_left;

        // 邊緣檢測：只在按鈕從未按下變為按下時觸發
        if (xNow && !prevX) {
            shooterOn = true;
            isHighVelocityMode = true;  // 切換到遠距離模式
        }

        if (dpadLeftNow && !prevDpadLeft) {
            shooterOn = true;
            isHighVelocityMode = false;  // 切換到近距離模式
        }

        // 緊急停止：立即關閉所有射球機構
        if (backNow && !prevBack) {
            shooterOn = false;
            feedEnabled = false;
            shooterMotor.setVelocity(0);
            feederServo.setPower(0);
        }

        // 更新按鈕狀態
        prevX = xNow;
        prevBack = backNow;
        prevDpadLeft = dpadLeftNow;

        // 根據狀態設定 Shooter 速度
        if (shooterOn) {
            double targetRPM = isHighVelocityMode ? HIGH_RPM : LOW_RPM;
            shooterMotor.setVelocity(CalculateTargetVelocity(targetRPM));
        } else {
            shooterMotor.setVelocity(0);
        }
    }

    /**
     * 🔄 處理自動送球控制
     * - Y 按住：遠距離送球 (需達速度)
     * - D-pad Up 按住：近距離送球 (需達速度)
     * - 鬆開按鈕：自動吐球 (防止卡球)
     */
    private void handleFeederControls() {
        boolean yHeld = gamepad1.y;
        boolean dpadUpHeld = gamepad1.dpad_up;
        double currentVelocity = shooterMotor.getVelocity();

        if (yHeld) {
            // 遠距離模式：檢查是否達到 HIGH_RPM
            handleFeedLogic(currentVelocity, CalculateTargetVelocity(HIGH_RPM), HIGH_VELOCITY_TOLERANCE);
        } else if (dpadUpHeld) {
            // 近距離模式：檢查是否達到 LOW_RPM
            handleFeedLogic(currentVelocity, CalculateTargetVelocity(LOW_RPM), LOW_VELOCITY_TOLERANCE);
        } else {
            // 沒有按送球按鈕：持續吐球 (防止球卡在 feeder)
            feederServo.setPower(FEEDER_OUTTAKE_POWER);
            feedEnabled = false;
        }
    }

    /**
     * ⚙️ 送球邏輯 (帶遲滯控制)
     *
     * 遲滯控制 (Hysteresis)：
     * - 開啟送球：當前速度 >= 目標速度
     * - 停止送球：當前速度 <= 目標速度 - 容差
     *
     * 目的：防止速度在臨界點震盪時 feeder 頻繁開關
     *
     * @param currentVelocity 當前實際速度 (ticks/s)
     * @param targetVelocity  目標速度門檻 (ticks/s)
     * @param tolerance       速度容差 (ticks/s)
     */
    private void handleFeedLogic(double currentVelocity, double targetVelocity, double tolerance) {
        if (!shooterOn) {
            feederServo.setPower(0.0);
            feedEnabled = false;
            return;
        }

        // 遲滯控制狀態機
        if (!feedEnabled && currentVelocity >= targetVelocity) {
            // 狀態轉換：速度達標 → 開始送球
            feedEnabled = true;
        } else if (feedEnabled && currentVelocity <= targetVelocity - tolerance) {
            // 狀態轉換：速度下降太多 → 停止送球
            feedEnabled = false;
        }

        // 根據狀態設定 feeder 功率
        feederServo.setPower(feedEnabled ? FEEDER_FEED_POWER : 0.0);
    }

    /**
     * 🔵 處理 Intake 控制
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
     * 📊 計算當前實際 RPM
     * @return 當前轉速 (每分鐘轉數)
     */
    private double CalculateCurrentRPM() {
        if (!shooterOn) return 0.0;

        double ticksPerSecond = shooterMotor.getVelocity();
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    /**
     * 🔢 將 RPM 轉換為馬達速度 (ticks/s)
     *
     * 公式：velocity = (rpm / 60) × ticks_per_rev
     *
     * @param rpm 目標轉速 (每分鐘轉數)
     * @return 對應的速度 (ticks per second)
     */
    private double CalculateTargetVelocity(double rpm) {
        return (rpm / 60.0) * SHOOTER_TICKS_PER_REV;
    }

    /**
     * 📺 更新 Driver Station 螢幕顯示
     */
    private void updateTelemetry() {
        double currentVelocity = shooterMotor.getVelocity();
        double targetRPM = isHighVelocityMode ? HIGH_RPM : LOW_RPM;
        double targetVelocity = CalculateTargetVelocity(targetRPM);
        double actualRPM = CalculateCurrentRPM();
        double error = currentVelocity - targetVelocity;

        // ===== 系統狀態 =====
        telemetry.addData("Shooter", shooterOn ? "🟢 運轉中" : "🔴 停止");
        telemetry.addData("模式", isHighVelocityMode ? "🚀 遠距離" : "🎯 近距離");
        telemetry.addData("Feeder", feedEnabled ? "✅ 送球中" : "⏸️ 待命");

        // ===== 速度資訊 =====
        telemetry.addData("🎯 目標RPM", String.format("%.0f RPM", targetRPM));
        telemetry.addData("📊 實際RPM", String.format("%.0f RPM", actualRPM));
        telemetry.addData("📉 Error", String.format("%+.1f ticks/s", error));
        telemetry.addData("✅ 達到目標速度", feedEnabled ? "是" : "否");

        // ===== 控制輸入 =====
        telemetry.addData("🚀 Y (遠距送球)", gamepad1.y ? "按住" : "-");
        telemetry.addData("🎯 ⬆️ (近距送球)", gamepad1.dpad_up ? "按住" : "-");

        telemetry.update();
    }

    /**
     * ⛔ 停止所有馬達 (程式結束時呼叫)
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
