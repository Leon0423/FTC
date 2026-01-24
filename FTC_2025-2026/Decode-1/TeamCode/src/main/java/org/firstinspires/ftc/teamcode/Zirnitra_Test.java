package org.firstinspires.ftc.teamcode;

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

    // ShooterAngle Servo 參數
    private static final double SHOOTERANGLE_INIT_POSITION = 0.16;
    private static final double SHOOTERANGLE_MIN_LIMIT = 0.0;
    private static final double SHOOTERANGLE_MAX_LIMIT = 0.16;
    private static final double SHOOTERANGLE_STEP = 0.01;

    // 速度設定 (ticks/second)
    private static final double LOW_VELOCITY = 1000.0;
    private static final double HIGH_VELOCITY = 2000.0;
    private static final double VELOCITY_TOLERANCE = 40.0;

    // 馬達功率設定
    private static final double INTAKE_POWER = 0.8;
    private static final double TRIGGER_THRESHOLD = 0.2;

    // 速度設定 (RPM)
    private static final double LOW_VELOCITY_RPM = 2142.0;   // 約1000 ticks/s
    private static final double HIGH_VELOCITY_RPM = 4285.0;  // 約2000 ticks/s
    private static final double VELOCITY_TOLERANCE_RPM = 85.0;

    // ═══════════════════════════════════════════════════════════════
    // 狀態變數 (State Variables)
    // ═══════════════════════════════════════════════════════════════

    private double currentShooterAngle = SHOOTERANGLE_INIT_POSITION;
    private boolean shooterOn = false;
    private boolean feedEnabled = false;
    private boolean isHighVelocityMode = true;

    // 按鍵邊緣檢測 (Button Edge Detection)
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    // ═══════════════════════════════════════════════════════════════
    // 主程式 (Main Program)
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addData("Controls", "X=High, DpadLeft=Low, RB=Stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 處理所有控制輸入
            handleDriveControls();
            handleShooterControls();
            handleShooterAngleControls();
            handleTriggerControls();
            handleIntakeControls();

            // 更新遙測資料
            updateTelemetry();
        }

        stopAllMotors();
    }

    // ═══════════════════════════════════════════════════════════════
    // 硬體初始化 (Hardware Initialization)
    // ═══════════════════════════════════════════════════════════════

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
        shooter_Left.setDirection(DcMotorSimple.Direction.REVERSE);

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

    // ═══════════════════════════════════════════════════════════════
    // 控制處理 (Control Handlers)
    // ═══════════════════════════════════════════════════════════════

    /**
     * 處理麥克納姆輪底盤控制
     * 左搖桿Y軸：前後移動
     * 左搖桿X軸：旋轉
     * 右搖桿X軸：左右平移
     */
    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;  // 前後
        double strafe = gamepad1.right_stick_x;   // 平移
        double rotate = gamepad1.left_stick_x;    // 旋轉
        double fr, fl, br, bl, scale;

        fr = forward - rotate - strafe;
        fl = forward + rotate + strafe;
        br = forward - rotate + strafe;
        bl = forward + rotate - strafe;

        scale = scaling_power(fr, fl, br, bl);

        FR.setPower(fr/scale);
        FL.setPower(fl/scale);
        BR.setPower(br/scale);
        BL.setPower(bl/scale);
    }

    public double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if(max <= 1) {
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
        }

        // 啟動低速模式
        if (dpadLeftPressed) {
            shooterOn = true;
            isHighVelocityMode = false;
        }

        // 關閉發射器並重置角度
        if (backPressed) {
            shooterOn = false;
            feedEnabled = false;
            resetShooterAngle();
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
            feedEnabled = Math.abs(leftRPM - targetRPM) <= VELOCITY_TOLERANCE_RPM &&
                    Math.abs(rightRPM - targetRPM) <= VELOCITY_TOLERANCE_RPM;
        } else {
            shooter_Left.setVelocity(0);
            shooter_Right.setVelocity(0);
            feedEnabled = false;
        }
    }


    /**
     * 檢查速度是否在目標範圍內
     */
    private boolean isVelocityOnTarget(double current, double target) {
        return Math.abs(current - target) <= VELOCITY_TOLERANCE;
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
     * 更新角度伺服馬達位置
     */
    private void updateShooterAngleServos() {
        shooterAngle_Right.setPosition(currentShooterAngle);
        shooterAngle_Left.setPosition(currentShooterAngle);
    }

    /**
     * 重置發射角度到初始位置
     */
    private void resetShooterAngle() {
        currentShooterAngle = SHOOTERANGLE_INIT_POSITION;
        updateShooterAngleServos();
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
            intake_2.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intake_1.setPower(0);
            intake_2.setPower(0);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // 遙測與工具 (Telemetry & Utilities)
    // ═══════════════════════════════════════════════════════════════

    private double ticksToRPM(double ticksPerSecond) {
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    private void updateTelemetry() {
        double leftRPM = ticksToRPM(shooter_Left.getVelocity());
        double rightRPM = ticksToRPM(shooter_Right.getVelocity());
        double targetRPM = shooterOn ? (isHighVelocityMode ? HIGH_VELOCITY_RPM : LOW_VELOCITY_RPM) : 0;

        telemetry.addLine("══════ 系統狀態 ══════");
        telemetry.addData("Shooter", shooterOn ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("模式", isHighVelocityMode ? "遠距離 (HIGH)" : "近距離 (LOW)");
        telemetry.addData("可發射", feedEnabled ? "✓ YES" : "✗ NO");

        telemetry.addLine("══════ 速度資訊 ══════");
        telemetry.addData("目標速度", "%.0f RPM", targetRPM);
        telemetry.addData("左馬達", "%.0f RPM", leftRPM);
        telemetry.addData("右馬達", "%.0f RPM", rightRPM);

        telemetry.addLine("══════ Servo 狀態 ══════");
        telemetry.addData("Trigger", "%.3f", Trigger.getPosition());
        telemetry.addData("ShooterAngle", "%.3f [%.2f ~ %.2f]",
                currentShooterAngle, SHOOTERANGLE_MIN_LIMIT, SHOOTERANGLE_MAX_LIMIT);

        telemetry.addLine("══════ 操作說明 ══════");
        telemetry.addData("發射", "X=遠, DpadLeft=近, RB=停");
        telemetry.addData("角度", "DpadUp/Down 微調");
        telemetry.addData("intake", "A=開, B=關");

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
