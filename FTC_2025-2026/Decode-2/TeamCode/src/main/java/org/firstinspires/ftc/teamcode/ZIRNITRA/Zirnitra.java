package org.firstinspires.ftc.teamcode.ZIRNITRA;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zirnitra_TeleOpMode")
public class Zirnitra extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════
    // 硬體宣告 (Hardware Declarations)
    // ═══════════════════════════════════════════════════════════════

    // 底盤馬達 (Drive Motors)
    private DcMotor FR, BR, FL, BL;

    // 進球機構 (Intake Motors)
    private DcMotor intake_1, intake_2;

    // 發射器馬達 (Shooter Motors)
    private DcMotorEx shooter_Right, shooter_Left;

    // 伺服馬達 (Servos)
    private Servo shooterAngle_Right, shooterAngle_Left, Trigger;

    // ═══════════════════════════════════════════════════════════════
    // 常數設定 (Constants)
    // ═══════════════════════════════════════════════════════════════

    // Shooter 編碼器參數
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Trigger Servo 參數
    private static final double TRIGGER_INIT_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.235;

    // ShooterAngle Servo 參數（固定角度模式）
    private static final double SHOOTERANGLE_INIT_POSITION = 0.16;

    // 速度設定 (RPM)
    private static final double LOW_VELOCITY_RPM = 3750.0;
    private static final double HIGH_VELOCITY_RPM = 4400.0;
    private static final double VELOCITY_TOLERANCE_RPM = 150.0;

    // 馬達功率設定
    private static final double INTAKE_POWER = 0.5;
    private static final double TRIGGER_THRESHOLD = 0.2;

    // RPM 穩定檢測
    private static final int RPM_STABLE_COUNT_REQUIRED = 3;

    // ═══════════════════════════════════════════════════════════════
    // 狀態變數 (State Variables)
    // ═══════════════════════════════════════════════════════════════

    private boolean shooterOn = false;
    private boolean feedEnabled = false;
    private boolean isHighVelocityMode = true;
    private int rpmStableCounter = 0;

    // 按鍵邊緣檢測 (Button Edge Detection)
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevDpadLeft = false;

    // ═══════════════════════════════════════════════════════════════
    // 主程式 (Main Program)
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void runOpMode() throws InterruptedException{
        initializeHardware();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addLine();
        telemetry.addLine("【發射器控制 - Gamepad1】");
        telemetry.addData("X", "啟動高速模式 (遠距離)");
        telemetry.addData("Dpad Left", "啟動低速模式 (近距離)");
        telemetry.addData("Right Bumper", "關閉發射器");
        telemetry.addData("Right Trigger", "發射 (需達目標速度)");

        telemetry.addLine();
        telemetry.addLine("【進球機構控制 - Gamepad1】");
        telemetry.addData("A", "啟動 Intake_1");
        telemetry.addData("B", "停止 Intake_1");
        telemetry.addData("Dpad Up", "Intake_2 正轉");
        telemetry.addData("Dpad Down", "Intake_2 反轉");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 處理所有控制輸入
            handleDriveControls();
            handleShooterControls();
            handleTriggerControls();
            handleIntakeControls();
            handleIntake_2Controls();

            // 更新遙測資料
            updateTelemetry();
        }

        stopAllMotors();
    }

    // 硬體初始化 (Hardware Initialization)
    private void initializeHardware() {
        initializeDriveMotors();
        initializeIntakeMotors();
        initializeShooterMotors();
        initializeServos();
        stopAllMotors();
    }
    private void initializeDriveMotors() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        // 設定方向（全部正向）
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        // 設定煞車模式
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void initializeIntakeMotors() {
        intake_1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");

        intake_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    private void initializeShooterMotors() {
        shooter_Right = hardwareMap.get(DcMotorEx.class, "shooter_Right");
        shooter_Left = hardwareMap.get(DcMotorEx.class, "shooter_Left");

        shooter_Right.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_Left.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 重設編碼器並使用編碼器模式
        shooter_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void initializeServos() {
        // Trigger Servo
        Trigger = hardwareMap.get(Servo.class, "Trigger");
        Trigger.setDirection(Servo.Direction.REVERSE);
        Trigger.setPosition(TRIGGER_INIT_POSITION);

        // ShooterAngle Servos
        shooterAngle_Right = hardwareMap.get(Servo.class, "shooterAngle_Right");
        shooterAngle_Left = hardwareMap.get(Servo.class, "shooterAngle_Left");
        shooterAngle_Right.setDirection(Servo.Direction.FORWARD);
        shooterAngle_Left.setDirection(Servo.Direction.REVERSE);
        shooterAngle_Right.setPosition(SHOOTERANGLE_INIT_POSITION);
        shooterAngle_Left.setPosition(SHOOTERANGLE_INIT_POSITION);
    }

    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;  // 前後
        double strafe = gamepad1.left_stick_x;    // 平移
        double rotate = gamepad1.right_stick_x;   // 旋轉
        double fr, fl, br, bl, scale;

        fr = forward - rotate - strafe;
        fl = forward + rotate + strafe;
        br = forward - rotate + strafe;
        bl = forward + rotate - strafe;

        scale = scaling_power(fr, fl, br, bl);

        FR.setPower(fr / scale);
        FL.setPower(fl / scale);
        BR.setPower(br / scale);
        BL.setPower(bl / scale);
    }
    private double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if (max <= 1) {
            max = 1;
        }
        return max;
    }

    /**
     * 處理發射器控制
     * X：啟動高速模式（遠距離）
     * Dpad Left：啟動低速模式（近距離）
     * Right Bumper：關閉發射器
     */
    private void handleShooterControls() {
        boolean xPressed = gamepad1.x && !prevX;
        boolean dpadLeftPressed = gamepad1.dpad_left && !prevDpadLeft;
        boolean backPressed = gamepad1.right_bumper && !prevBack;

        // 啟動高速模式
        if (xPressed) {
            shooterOn = true;
            isHighVelocityMode = true;
            intake_1.setPower(0);
            rpmStableCounter = 0;
        }

        // 啟動低速模式
        if (dpadLeftPressed) {
            shooterOn = true;
            isHighVelocityMode = false;
            intake_1.setPower(0);
            rpmStableCounter = 0;
        }

        // 關閉發射器
        if (backPressed) {
            shooterOn = false;
            feedEnabled = false;
            rpmStableCounter = 0;
        }

        // 更新按鍵狀態
        prevX = gamepad1.x;
        prevBack = gamepad1.right_bumper;
        prevDpadLeft = gamepad1.dpad_left;

        // 設定發射器速度並檢查是否達標
        updateShooterVelocity();
    }

    /**
     * 將 RPM 轉換為 ticks/second
     */
    private double rpmToTicks(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV) / 60.0;
    }

    /**
     * 更新發射器速度並檢查是否達到目標速度
     * 需要連續 3 次達標才能啟用發射
     */
    private void updateShooterVelocity() {
        if (shooterOn) {
            double targetRPM = isHighVelocityMode ? HIGH_VELOCITY_RPM : LOW_VELOCITY_RPM;
            double targetTicks = rpmToTicks(targetRPM);

            shooter_Left.setVelocity(targetTicks);
            shooter_Right.setVelocity(targetTicks);

            // 檢查是否達到目標速度 (使用RPM比較)
            double leftRPM = ticksToRPM(shooter_Left.getVelocity());
            double rightRPM = ticksToRPM(shooter_Right.getVelocity());
            boolean isStable = Math.abs(leftRPM - targetRPM) <= VELOCITY_TOLERANCE_RPM &&
                    Math.abs(rightRPM - targetRPM) <= VELOCITY_TOLERANCE_RPM;

            // 更新穩定計數器
            if (isStable) {
                rpmStableCounter++;
                if (rpmStableCounter >= RPM_STABLE_COUNT_REQUIRED) {
                    feedEnabled = true;
                }
            } else {
                rpmStableCounter = 0;
                feedEnabled = false;
            }
        } else {
            shooter_Left.setVelocity(0);
            shooter_Right.setVelocity(0);
            feedEnabled = false;
            rpmStableCounter = 0;
        }
    }

    private void handleIntake_2Controls(){
        if (gamepad1.dpad_up) {
            intake_2.setPower(INTAKE_POWER);
        } else if (gamepad1.dpad_down) {
            intake_2.setPower(-INTAKE_POWER);
        } else {
            intake_2.setPower(0);
        }
    }

    /**
     * 處理觸發器控制
     * 只有當發射器達到目標速度時才能發射
     */
    private void handleTriggerControls() {
        boolean canFire = feedEnabled && gamepad1.right_trigger > TRIGGER_THRESHOLD;
        Trigger.setPosition(canFire ? TRIGGER_FIRE_POSITION : TRIGGER_INIT_POSITION);
    }

    /**
     * 處理進球機構控制
     * A：啟動進球
     * B：停止進球
     */
    private void handleIntakeControls() {
        if (gamepad1.a) {
            intake_1.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intake_1.setPower(0);
        }
    }

    // 遙測與工具 (Telemetry & Utilities)
    private double ticksToRPM(double ticksPerSecond) {
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    private void updateTelemetry() {
        double leftRPM = ticksToRPM(shooter_Left.getVelocity());
        double rightRPM = ticksToRPM(shooter_Right.getVelocity());
        double targetRPM = shooterOn ? (isHighVelocityMode ? HIGH_VELOCITY_RPM : LOW_VELOCITY_RPM) : 0;
        double leftError = leftRPM - targetRPM;
        double rightError = rightRPM - targetRPM;

        telemetry.addLine("══════ 系統狀態 ══════");
        telemetry.addData("Shooter", shooterOn ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("模式", isHighVelocityMode ? "遠距離 (HIGH)" : "近距離 (LOW)");
        telemetry.addData("可發射", feedEnabled ? "✓ YES" : "✗ NO");
        telemetry.addData("穩定次數", "%d / %d", rpmStableCounter, RPM_STABLE_COUNT_REQUIRED);

        telemetry.addLine("══════ 速度資訊 ══════");
        telemetry.addData("目標RPM", "%.0f RPM", targetRPM);
        telemetry.addData("左馬達", "%.0f RPM (誤差: %.0f)", leftRPM, leftError);
        telemetry.addData("右馬達", "%.0f RPM (誤差: %.0f)", rightRPM, rightError);

        telemetry.addLine("══════ Servo 狀態 ══════");
        telemetry.addData("Trigger", "%.3f", Trigger.getPosition());
        telemetry.addData("ShooterAngle", "%.3f", shooterAngle_Right.getPosition());

        telemetry.addLine("══════ 操作說明 ══════");
        telemetry.addData("發射", "X=遠, DpadLeft=近, RB=停");
        telemetry.addData("Intake", "A=開, B=關");

        telemetry.update();
    }

    private void stopAllMotors() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        shooter_Left.setVelocity(0);
        shooter_Right.setVelocity(0);
        intake_1.setPower(0);
        intake_2.setPower(0);
    }
}
