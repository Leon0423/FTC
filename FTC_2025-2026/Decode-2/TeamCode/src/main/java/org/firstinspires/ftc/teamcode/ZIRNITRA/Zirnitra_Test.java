package org.firstinspires.ftc.teamcode.ZIRNITRA;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zirnitra_Test")
public class Zirnitra_Test extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════
    // 硬體宣告 (Hardware Declarations)
    // ═══════════════════════════════════════════════════════════════

    // 底盤馬達 (Drive Motors)
    private DcMotor FR, BR, FL, BL;

    // 發射器馬達 (Shooter Motors)
    private DcMotorEx shooter_Right, shooter_Left;

    // 發射器伺服馬達 (Shooter Servos)
    private Servo shooterAngle_Right, shooterAngle_Left, Trigger;

    // 進球機構 (Intake Motors)
    private DcMotor intake_1, intake_2;

    // ═══════════════════════════════════════════════════════════════
    // 常數設定 (Constants)
    // ═══════════════════════════════════════════════════════════════

    // 底盤參數
    // (無特殊常數)

    // 發射器參數
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double LOW_RPM = 2000.0;
    private static final double HIGH_RPM = 4200.0;
    private static final double RPM_TOLERANCE = 50.0;
    private static final double RPM_ADJUST_STEP = 50.0;
    private static final double RPM_MAX_LIMIT = 5800.0;

    // 發射角度參數
    private static final double SHOOTERANGLE_INIT_POSITION = 0.16;
    private static final double SHOOTERANGLE_MIN_LIMIT = 0.0;
    private static final double SHOOTERANGLE_MAX_LIMIT = 0.16;
    private static final double SHOOTERANGLE_STEP = 0.01;

    // Trigger 參數
    private static final double TRIGGER_INIT_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.235;
    private static final double TRIGGER_THRESHOLD = 0.2;

    // 進球機構參數
    private static final double INTAKE_POWER = 0.8;

    // ═══════════════════════════════════════════════════════════════
    // 狀態變數 (State Variables)
    // ═══════════════════════════════════════════════════════════════

    // 底盤狀態
    // (無特殊狀態變數)

    // 發射器狀態
    private boolean shooterOn = false;
    private boolean feedEnabled = false;
    private boolean isHighVelocityMode = true;
    private double targetRPM = 0;
    private double currentShooterAngle = SHOOTERANGLE_INIT_POSITION;

    // 按鍵邊緣檢測
    private boolean prevX = false;
    private boolean prevDpadLeft = false;
    private boolean prevBack = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevG2DpadUp = false;
    private boolean prevG2DpadDown = false;

    // ═══════════════════════════════════════════════════════════════
    // 主程式 (Main Program)
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ===== 底盤控制 =====
            handleDriveControls();

            // ===== 發射系統控制 =====
            handleShooterControls();
            handleShooterAngleControls();
            handleTriggerControls();
            handleRPMAdjustment();

            // ===== 進球機構控制 =====
            handleIntakeControls();

            // 更新遙測
            updateTelemetry();
        }

        stopAllMotors();
    }

    // ═══════════════════════════════════════════════════════════════
    // 硬體初始化 (Hardware Initialization)
    // ═══════════════════════════════════════════════════════════════
    
    private void initializeHardware() {
        initializeDriveMotors();
        initializeShooterMotors();
        initializeServos();
        initializeIntakeMotors();
        stopAllMotors();
    }
    private void initializeDriveMotors() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void initializeShooterMotors() {
        shooter_Right = hardwareMap.get(DcMotorEx.class, "shooter_Right");
        shooter_Left = hardwareMap.get(DcMotorEx.class, "shooter_Left");

        shooter_Right.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_Left.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void initializeServos() {
        Trigger = hardwareMap.get(Servo.class, "Trigger");
        Trigger.setDirection(Servo.Direction.REVERSE);
        Trigger.setPosition(TRIGGER_INIT_POSITION);

        shooterAngle_Right = hardwareMap.get(Servo.class, "shooterAngle_Right");
        shooterAngle_Left = hardwareMap.get(Servo.class, "shooterAngle_Left");
        shooterAngle_Right.setDirection(Servo.Direction.FORWARD);
        shooterAngle_Left.setDirection(Servo.Direction.REVERSE);
        shooterAngle_Right.setPosition(SHOOTERANGLE_INIT_POSITION);
        shooterAngle_Left.setPosition(SHOOTERANGLE_INIT_POSITION);
    }
    private void initializeIntakeMotors() {
        intake_1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");

        intake_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // ═══════════════════════════════════════════════════════════════
    // 底盤控制 (Drive Control)
    // ═══════════════════════════════════════════════════════════════

    /**
     * 處理麥克納姆輪底盤控制
     * 左搖桿Y軸：前後移動
     * 左搖桿X軸：左右平移
     * 右搖桿X軸：旋轉
     */
    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double fr = forward - rotate - strafe;
        double fl = forward + rotate + strafe;
        double br = forward - rotate + strafe;
        double bl = forward + rotate - strafe;

        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        double scale = Math.max(max, 1.0);

        FR.setPower(fr / scale);
        FL.setPower(fl / scale);
        BR.setPower(br / scale);
        BL.setPower(bl / scale);
    }

    // ═══════════════════════════════════════════════════════════════
    // 發射系統控制 (Shooter System Control)
    // ═══════════════════════════════════════════════════════════════

    /**
     * 處理發射器啟動模式控制
     * X：啟動高速模式（遠距離，4200 RPM）
     * Dpad Left：啟動低速模式（近距離，2000 RPM）
     * Right Bumper：關閉發射器並重置角度
     */
    private void handleShooterControls() {
        boolean xPressed = gamepad1.x && !prevX;
        boolean dpadLeftPressed = gamepad1.dpad_left && !prevDpadLeft;
        boolean backPressed = gamepad1.right_bumper && !prevBack;

        if (xPressed) {
            shooterOn = true;
            isHighVelocityMode = true;
            targetRPM = HIGH_RPM;
        }

        if (dpadLeftPressed) {
            shooterOn = true;
            isHighVelocityMode = false;
            targetRPM = LOW_RPM;
        }

        if (backPressed) {
            shooterOn = false;
            feedEnabled = false;
            targetRPM = 0;
            resetShooterAngle();
        }

        prevX = gamepad1.x;
        prevDpadLeft = gamepad1.dpad_left;
        prevBack = gamepad1.right_bumper;

        updateShooterVelocity();
    }

    /**
     * 處理發射角度微調
     * Dpad Up：降低角度值（射更遠）
     * Dpad Down：提高角度值（射更近）
     */
    private void handleShooterAngleControls() {
        boolean dpadUpPressed = gamepad1.dpad_up && !prevDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !prevDpadDown;

        if (dpadUpPressed) {
            currentShooterAngle = Math.max(currentShooterAngle - SHOOTERANGLE_STEP, SHOOTERANGLE_MIN_LIMIT);
            updateShooterAngleServos();
        }

        if (dpadDownPressed) {
            currentShooterAngle = Math.min(currentShooterAngle + SHOOTERANGLE_STEP, SHOOTERANGLE_MAX_LIMIT);
            updateShooterAngleServos();
        }

        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
    }

    /**
     * 處理觸發器控制
     * Right Trigger：發射（需發射器達到目標轉速）
     */
    private void handleTriggerControls() {
        boolean canFire = feedEnabled && gamepad1.right_trigger > TRIGGER_THRESHOLD;
        Trigger.setPosition(canFire ? TRIGGER_FIRE_POSITION : TRIGGER_INIT_POSITION);
    }

    /**
     * 處理 RPM 精細調整（Gamepad 2）
     * Dpad Up：增加 50 RPM
     * Dpad Down：減少 50 RPM（範圍：0~5800 RPM）
     */
    private void handleRPMAdjustment() {
        boolean g2DpadUpPressed = gamepad2.dpad_up && !prevG2DpadUp;
        boolean g2DpadDownPressed = gamepad2.dpad_down && !prevG2DpadDown;

        if (g2DpadUpPressed) {
            targetRPM += RPM_ADJUST_STEP;
        }

        if (g2DpadDownPressed) {
            targetRPM -= RPM_ADJUST_STEP;
        }

        targetRPM = Math.max(0, Math.min(targetRPM, RPM_MAX_LIMIT));

        prevG2DpadUp = gamepad2.dpad_up;
        prevG2DpadDown = gamepad2.dpad_down;
    }

    /**
     * 更新發射器速度並檢查是否達到目標速度
     */
    private void updateShooterVelocity() {
        if (shooterOn) {
            double targetTicks = (targetRPM * SHOOTER_TICKS_PER_REV) / 60.0;

            shooter_Left.setVelocity(targetTicks);
            shooter_Right.setVelocity(targetTicks);

            double leftRPM = (shooter_Left.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0;
            double rightRPM = (shooter_Right.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0;
            feedEnabled = Math.abs(leftRPM - targetRPM) <= RPM_TOLERANCE &&
                    Math.abs(rightRPM - targetRPM) <= RPM_TOLERANCE;
        } else {
            shooter_Left.setVelocity(0);
            shooter_Right.setVelocity(0);
            feedEnabled = false;
        }
    }

    private void updateShooterAngleServos() {
        shooterAngle_Right.setPosition(currentShooterAngle);
        shooterAngle_Left.setPosition(currentShooterAngle);
    }

    private void resetShooterAngle() {
        currentShooterAngle = SHOOTERANGLE_INIT_POSITION;
        updateShooterAngleServos();
    }

    /**
     * 處理進球機構控制
     * A：啟動進球機構
     * B：停止進球機構
     */
    private void handleIntakeControls() {
        if (gamepad1.a) {
            intake_1.setPower(INTAKE_POWER);
            intake_2.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intake_1.setPower(0);
            intake_2.setPower(0);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // 遙測與工具 (Telemetry & Utilities)
    // ═══════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        double leftRPM = (shooter_Left.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0;
        double rightRPM = (shooter_Right.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0;
        double avgRPM = (leftRPM + rightRPM) / 2.0;
        double error = avgRPM - targetRPM;

        telemetry.addLine("══════ 系統狀態 ══════");
        telemetry.addData("模式", isHighVelocityMode ? "遠距離 (HIGH)" : "近距離 (LOW)");
        telemetry.addData("可發射", feedEnabled ? "✓ YES" : "✗ NO");

        telemetry.addLine("══════ 速度資訊 ══════");
        telemetry.addData("目標 RPM", "%.2f", targetRPM);
        telemetry.addData("左馬達", "%.2f RPM", leftRPM);
        telemetry.addData("右馬達", "%.2f RPM", rightRPM);
        telemetry.addData("Error", "%+.1f RPM", error);

        telemetry.addLine("══════ Servo 狀態 ══════");
        telemetry.addData("Trigger", "%.3f", Trigger.getPosition());
        telemetry.addData("ShooterAngle", "%.3f", currentShooterAngle);

        telemetry.addLine("══════ 操作說明 ══════");
        telemetry.addData("發射器", "X=遠, DpadLeft=近, RB=停");
        telemetry.addData("角度", "DpadUp/Down");
        telemetry.addData("RPM", "GP2: DpadUp/Down ±50");
        telemetry.addData("進球", "A=開, B=關");

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
