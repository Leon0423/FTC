package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "team_1_final", group = "TeleOp")
public class team_1_test extends LinearOpMode {

    // ===== Drive Motors =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== Shooter / Feeder =====
    private DcMotorEx shooterMotor;      // goBILDA 5202-0002-0001 (28 CPR)
    private CRServo feederServo;       // continue servo

    // ===== Intake =====
    private DcMotor intakeMotor;       // 312 RPM intake

    // ===== 參數 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // 每圈 28 個 ticks

    // ===== PIDF =====
    private static final double SHOOTER_P = 230.0;   // 你的 P 值
    private static final double SHOOTER_I = 0.0;    // 你的 I 值
    private static final double SHOOTER_D = 0.0;    // 你的 D 值
    private static final double SHOOTER_F = 11.5;   // 你的 F 值


    // 送球門檻
    private static final double LOW_RPM = 2000;  // 近距離射球速度
    private static final double HIGH_RPM = 4500; // 遠距離射球速度

    // ===== 速度容差 (用於判斷是否達標) =====
    private static final double HIGH_VELOCITY_TOLERANCE = 20;  // 高速模式容差
    private static final double LOW_VELOCITY_TOLERANCE = 20;   // 低速模式容差

    // ===== Servo 功率設定 =====
    private static final double FEEDER_OUTTAKE_POWER = 1.0;  // 吐球功率
    private static final double FEEDER_FEED_POWER = -1.0;  // 送球功率
    private static final double INTAKE_POWER = 0.8;  // 吸球功率

    // ===== 機構狀態 =====
    private boolean shooterOn = false;
    private boolean feedEnabled = false;

    // ===== 按鍵邊緣檢測 (防止單次按壓多次觸發) =====
    private boolean prevX = false;           // X 按鈕前一幀狀態
    private boolean prevBack = false;        // Back 按鈕前一幀狀態
    private boolean prevDpadLeft = false;    // D-pad Left 前一幀狀態
    private boolean isHighVelocityMode = true;  // 當前速度模式

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== 初始化硬體 =====
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // ===== 主循環 =====
        while (opModeIsActive()) {
            handleDriveControls();
            handleShooterControls();
            handleFeederControls();
            handleIntakeControls();
            updateTelemetry();
        }

        // ===== 停止所有馬達 =====
        stopAllMotors();
    }

    private void initializeHardware() {
        // 底盤馬達

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // 左邊馬達反轉
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 底盤馬達設定
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 射球與進球機構
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Shooter 設定
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 設定 PIDF 參數
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // Intake 設定
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 全部停止
        stopAllMotors();
    }

    private void handleDriveControls() {
        double y  = gamepad1.left_stick_y;
        double rx  =  gamepad1.left_stick_x;
        double x =  gamepad1.right_stick_x;

        frontLeft.setPower(-y + x + rx);
        frontRight.setPower(-y - x - rx);
        backLeft.setPower(y - x + rx);
        backRight.setPower(y + x - rx);
    }

    private void handleShooterControls() {
        boolean xNow = gamepad1.x;
        boolean backNow = gamepad1.right_bumper;

        // X 按鈕：啟動 shooter（遠距離）
        if (xNow && !prevX) {
            shooterOn = true;
            isHighVelocityMode = true;
        }

        // D-pad Left：啟動 shooter（近距離）
        boolean dpadLeftNow = gamepad1.dpad_left;
        if (dpadLeftNow && !prevDpadLeft) {
            shooterOn = true;
            isHighVelocityMode = false;
        }

        // Right Bumper：緊急停止所有射球機構
        if (backNow && !prevBack) {
            shooterOn = false;
            feedEnabled = false;
            shooterMotor.setVelocity(0);
            feederServo.setPower(0);
        }

        // 更新按鈕狀態（用於下一幀的邊緣檢測）
        prevX = xNow;
        prevBack = backNow;
        prevDpadLeft = dpadLeftNow;

        // 設定 shooter 功率
        if (shooterOn) {
            if (isHighVelocityMode) {
                shooterMotor.setVelocity(CalculateTargetVelocity(HIGH_RPM));
            } else {
                shooterMotor.setVelocity(CalculateTargetVelocity(LOW_RPM));
            }
        } else {
            shooterMotor.setVelocity(0);
        }
    }

    private void handleFeederControls() {
        boolean yHeld = gamepad1.y;
        boolean dpadUpHeld = gamepad1.dpad_up;

        // 取得當前實際速度
        double currentVelocity = shooterMotor.getVelocity();

        // 判斷是否有任何送球按鈕被按住
        if (yHeld) {
            // Y 按住：遠距離模式
            handleFeedLogic(currentVelocity, CalculateTargetVelocity(HIGH_RPM), HIGH_VELOCITY_TOLERANCE);
        } else if (dpadUpHeld) {
            // D-pad Up 按住：近距離模式
            handleFeedLogic(currentVelocity, CalculateTargetVelocity(LOW_RPM), LOW_VELOCITY_TOLERANCE);
        } else {
            // 沒有按任何送球按鈕：持續吐球（防止卡球）
            feederServo.setPower(FEEDER_OUTTAKE_POWER);
            feedEnabled = false;
        }
    }

    /**
     * 送球邏輯（帶有遲滯控制，防止速度在臨界點附近時頻繁切換）
     * @param currentVelocity 當前實際速度
     * @param targetVelocity  目標速度門檻
     * @param tolerance       速度容差
     */
    private void handleFeedLogic(double currentVelocity, double targetVelocity, double tolerance) {
        if (!shooterOn) {
            // Shooter 沒開，不送球
            feederServo.setPower(0.0);
            feedEnabled = false;
            return;
        }

        // 遲滯控制：防止速度在臨界點震盪時頻繁開關
        if (!feedEnabled && currentVelocity >= targetVelocity) {
            // 速度達標，開始送球
            feedEnabled = true;
        } else if (feedEnabled && currentVelocity <= targetVelocity - tolerance) {
            // 速度掉太多，停止送球
            feedEnabled = false;
        }

        // 設定 feeder 功率
        feederServo.setPower(feedEnabled ? FEEDER_FEED_POWER : 0.0);
    }

    private void handleIntakeControls() {
        if (gamepad1.a) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intakeMotor.setPower(0);
        }
    }

    private double CalculateCurrentRPM() {
        if (!shooterOn) {
            return 0.0;
        }

        // getVelocity() 回傳的是 ticks per second
        double ticksPerSecond = shooterMotor.getVelocity();

        // 轉換成 RPM: (ticks/sec) / (ticks/rev) * 60 sec/min
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    /**
     * 將 RPM 轉換為 Shooter Motor 需要的 Velocity (ticks per second)
     * @param rpm 目標轉速 (每分鐘轉數)
     * @return 對應的 velocity (ticks per second)
     */
    private double CalculateTargetVelocity(double rpm) {
        // RPM -> ticks/sec 公式：
        // (rpm / 60) * ticks_per_rev
        return (rpm / 60.0) * SHOOTER_TICKS_PER_REV;
    }

    private void updateTelemetry() {
        double currentVelocity = shooterMotor.getVelocity();
        double targetVelocity = isHighVelocityMode ? HIGH_RPM : LOW_RPM;
        double rpm = CalculateCurrentRPM();
        boolean yHeld = gamepad1.y;
        boolean dpadUpHeld = gamepad1.dpad_up;

        // ===== 系統狀態 =====
        telemetry.addLine("══════ 系統狀態 ══════");
        telemetry.addData("Shooter", shooterOn ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("模式", isHighVelocityMode ? "遠距離" : "近距離");
        // ===== 速度資訊 =====
        telemetry.addLine("══════ 速度資訊 ══════");
        telemetry.addData("Error", String.format("%.1f ticks/s", currentVelocity - targetVelocity));
        telemetry.addData("目標 RPM", isHighVelocityMode ? HIGH_RPM : LOW_RPM);
        telemetry.addData("實際 RPM", String.format("%.0f", rpm));
        telemetry.addData("達到目標速度", feedEnabled ? "✓ YES" : "✗ NO");
        // ===== 控制輸入 =====
        telemetry.addLine("══════ 控制輸入 ══════");
        telemetry.addData("Y (遠距離)", yHeld ? "按住" : "-");
        telemetry.addData("DpadUp (近距離)", dpadUpHeld ? "按住" : "-");
        telemetry.update();
    }

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