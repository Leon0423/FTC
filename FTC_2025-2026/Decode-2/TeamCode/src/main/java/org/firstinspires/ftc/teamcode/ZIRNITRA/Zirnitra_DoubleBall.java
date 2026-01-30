package org.firstinspires.ftc.teamcode.ZIRNITRA;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Zirnitra_DoubleBall")
public class Zirnitra_DoubleBall extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════
    // 硬體宣告 (Hardware Declarations)
    // ═══════════════════════════════════════════════════════════════

    // 底盤馬達 (Drive Motors)
    private DcMotor FR, BR, FL, BL;

    // 發射器馬達 (Shooter Motors)
    private DcMotorEx shooter_Right, shooter_Left;

    // 發射器伺服馬達 (Shooter Servos)
    private Servo Trigger;

    // 進球機構 (Intake Motors)
    private DcMotor intake_1, intake_2;

    // ═══════════════════════════════════════════════════════════════
    // 射擊狀態機 (Shooting State Machine)
    // ═══════════════════════════════════════════════════════════════

    public enum ShootingState {
        IDLE,           // 待機
        SPIN_UP,        // 飛輪加速
        READY,          // 飛輪穩定，準備射擊
        FEEDING,        // 推球中
        WAIT_BALL,      // 等待下一顆球
        DONE            // 完成
    }

    private ShootingState shootingState = ShootingState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime stabilityTimer = new ElapsedTime();

    // ═══════════════════════════════════════════════════════════════
    // 常數設定 (Constants)
    // ═══════════════════════════════════════════════════════════════

    // 發射器參數
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double LOW_RPM = 3750;
    private static final double HIGH_RPM = 4400.0;
    private static final double RPM_TOLERANCE = 150.0;

    // 發射角度參數
    private static final double SHOOTERANGLE_INIT_POSITION = 0.68;

    // Trigger 參數
    private static final double TRIGGER_INIT_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.235;

    // 進球機構參數
    private static final double INTAKE_POWER = 0.5;
    private static final int TOTAL_BALLS = 2;  // 射2顆球後停止

    // 狀態機參數
    private static final int STABILITY_TIME = 50;  // 飛輪穩定時間 (ms)
    private static final int FEED_WAIT_TIME = 300;  // 每顆球之間等待時間 (ms)
    private static final int TRIGGER_FIRE_TIME = 200;  // 扳機射擊時間 (ms)

    // 發射器狀態
    private double targetRPM = 0;
    private double currentShooterAngle = SHOOTERANGLE_INIT_POSITION;
    private int ballCount = 0;
    private boolean isHighVelocityMode = true;

    // 按鍵邊緣檢測
    private boolean prevX = false;
    private boolean prevDpadLeft = false;
    private boolean prevBack = false;

    // ═══════════════════════════════════════════════════════════════
    // 主程式 (Main Program)
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addLine("【發射器控制 - Gamepad1】");
        telemetry.addData("X", "啟動遠距離模式 (HIGH RPM)");
        telemetry.addData("Dpad Left", "啟動近距離模式 (LOW RPM)");
        telemetry.addData("Right Bumper", "停止射擊");

        telemetry.addLine();
        telemetry.addLine("【進球機構控制 - Gamepad1】");
        telemetry.addData("A", "Intake_1 正轉");
        telemetry.addData("B", "Intake_1 反轉");

        telemetry.addLine();
        telemetry.addLine("【射擊流程】");
        telemetry.addData("1", "飛輪加速 → 穩定");
        telemetry.addData("2", "自動射擊 2 顆球");
        telemetry.addData("3", "完成後自動停止");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDriveControls();
            handleShooterControls();
            handleIntakeControls();
            updateShooting();

            updateTelemetry();
        }
    }

    // 硬體初始化 (Hardware Initialization)
    private void initializeHardware() {
        initializeDriveMotors();
        initializeShooterMotors();
        initializeServos();
        initializeIntakeMotors();
    }

    private void initializeDriveMotors() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeShooterMotors() {
        shooter_Right = hardwareMap.get(DcMotorEx.class, "shooter_Right");
        shooter_Left = hardwareMap.get(DcMotorEx.class, "shooter_Left");

        shooter_Right.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_Left.setDirection(DcMotorSimple.Direction.FORWARD);

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
    }

    private void initializeIntakeMotors() {
        intake_1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");
        intake_2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ═══════════════════════════════════════════════════════════════
    // 底盤控制 (Drive Controls)
    // ═══════════════════════════════════════════════════════════════

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
    // 射擊控制 (Shooter Controls)
    // ═══════════════════════════════════════════════════════════════

    private void handleShooterControls() {
        boolean xPressed = gamepad1.x && !prevX;
        boolean dpadLeftPressed = gamepad1.dpad_left && !prevDpadLeft;
        boolean backPressed = gamepad1.right_bumper && !prevBack;

        // X: 啟動遠距射擊
        if (xPressed && shootingState == ShootingState.IDLE) {
            startShooting(HIGH_RPM, true);
        }

        // DpadLeft: 啟動近距射擊
        if (dpadLeftPressed && shootingState == ShootingState.IDLE) {
            startShooting(LOW_RPM, false);
        }

        // RB: 停止射擊
        if (backPressed) {
            stopShooting();
        }

        prevX = gamepad1.x;
        prevDpadLeft = gamepad1.dpad_left;
        prevBack = gamepad1.right_bumper;
    }

    // 啟動射擊
    private void startShooting(double rpm, boolean isHighMode) {
        targetRPM = rpm;
        isHighVelocityMode = isHighMode;
        ballCount = 0;
        intake_1.setPower(0);

        shootingState = ShootingState.SPIN_UP;
        stateTimer.reset();
        stabilityTimer.reset();
    }

    // 停止射擊
    private void stopShooting() {
        shootingState = ShootingState.IDLE;
        targetRPM = 0;
        ballCount = 0;

        shooter_Left.setVelocity(0);
        shooter_Right.setVelocity(0);
        intake_2.setPower(0);
        Trigger.setPosition(TRIGGER_INIT_POSITION);
    }

    // 更新射擊狀態機
    private void updateShooting() {
        switch (shootingState) {
            case IDLE:
                break;

            case SPIN_UP:
                // 啟動飛輪和 intake_2
                double targetTicks = (targetRPM * SHOOTER_TICKS_PER_REV) / 60.0;
                shooter_Left.setVelocity(targetTicks);
                shooter_Right.setVelocity(targetTicks);
                intake_2.setPower(0.8);

                // 檢查飛輪是否穩定
                if (isFlywheelStable()) {
                    if (stabilityTimer.milliseconds() >= STABILITY_TIME) {
                        shootingState = ShootingState.READY;
                        stateTimer.reset();
                    }
                } else {
                    stabilityTimer.reset();
                }
                break;

            case READY:
                // 飛輪穩定，停止 intake_2 並開始射擊
                intake_2.setPower(0);
                Trigger.setPosition(TRIGGER_FIRE_POSITION);
                stateTimer.reset();
                shootingState = ShootingState.FEEDING;
                break;

            case FEEDING:
                // 等待扳機完成射擊動作
                if (stateTimer.milliseconds() >= TRIGGER_FIRE_TIME) {
                    Trigger.setPosition(TRIGGER_INIT_POSITION);  // 先將 Trigger 回到初始位置
                    ballCount++;

                    // 檢查是否完成所有球
                    if (ballCount >= TOTAL_BALLS) {
                        shootingState = ShootingState.DONE;
                        stateTimer.reset();
                    } else {
                        shootingState = ShootingState.WAIT_BALL;
                        stateTimer.reset();
                    }
                }
                break;

            case WAIT_BALL:
                // 確保 Trigger 在初始位置後，才啟動 intake_2 送下一顆球
                Trigger.setPosition(TRIGGER_INIT_POSITION);  // 確保 Trigger 回位
                intake_2.setPower(0.8);

                if (stateTimer.milliseconds() >= FEED_WAIT_TIME) {
                    if (isFlywheelStable()) {
                        shootingState = ShootingState.READY;
                    } else {
                        shootingState = ShootingState.SPIN_UP;
                        stabilityTimer.reset();
                    }
                }
                break;

            case DONE:
                // 完成後停止所有系統
                if (stateTimer.milliseconds() >= 200) {
                    stopShooting();
                }
                break;
        }
    }

    // 檢查飛輪是否穩定
    private boolean isFlywheelStable() {
        double leftRPM = Math.abs((shooter_Left.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0);
        double rightRPM = Math.abs((shooter_Right.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0);

        boolean leftStable = Math.abs(leftRPM - targetRPM) <= RPM_TOLERANCE;
        boolean rightStable = Math.abs(rightRPM - targetRPM) <= RPM_TOLERANCE;

        return leftStable && rightStable;
    }

    // ═══════════════════════════════════════════════════════════════
    // Intake 控制 (Intake Controls)
    // ═══════════════════════════════════════════════════════════════

    // intake_1: A=啟動, B=反轉, else=停止
    private void handleIntakeControls() {
        if (gamepad1.a) {
            intake_1.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intake_1.setPower(-1);
        } else {
            intake_1.setPower(0);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // 遙測 (Telemetry)
    // ═══════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        double leftRPM = Math.abs((shooter_Left.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0);
        double rightRPM = Math.abs((shooter_Right.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0);
        double leftError = leftRPM - targetRPM;
        double rightError = rightRPM - targetRPM;


        telemetry.addLine("══════ 系統狀態 ══════");
        telemetry.addData("射擊狀態", getShootingStateDisplay());
        telemetry.addData("模式", isHighVelocityMode ? "遠距離 (HIGH)" : "近距離 (LOW)");
        telemetry.addData("已射球數", "%d/%d", ballCount, TOTAL_BALLS);

        telemetry.addLine("══════ 速度資訊 ══════");
        telemetry.addData("目標 RPM", "%.2f", targetRPM);
        telemetry.addData("左馬達", "%.2f RPM(Error: %.2f RPM)", leftRPM, leftError);
        telemetry.addData("右馬達", "%.2f RPM(Error: %.2f RPM)", rightRPM, rightError);
        telemetry.addData("飛輪穩定", isFlywheelStable() ? "✓ YES" : "✗ NO");

        telemetry.addLine("══════ Servo 狀態 ══════");
        telemetry.addData("Trigger", "%.3f", Trigger.getPosition());
        telemetry.addData("ShooterAngle", "%.3f", currentShooterAngle);

        telemetry.addLine("══════ intake_2 狀態 ══════");
        telemetry.addData("Power", "%.2f", intake_2.getPower());

        telemetry.addLine("══════ 操作說明 ══════");
        telemetry.addData("發射器", "X=遠距, DpadLeft=近距, RB=停止");
        telemetry.addData("Intake", "A=啟動, B=反轉");

        telemetry.update();
    }

    private String getShootingStateDisplay() {
        switch (shootingState) {
            case IDLE:
                return "⏸ 待機";
            case SPIN_UP:
                return "⚡ 飛輪加速中...";
            case READY:
                return "✓ 準備射擊";
            case FEEDING:
                return "🎯 射擊中 (球 " + (ballCount + 1) + ")";
            case WAIT_BALL:
                return "⏳ 等待下一顆球";
            case DONE:
                return "✓ 射擊完成";
            default:
                return "未知狀態";
        }
    }

}

