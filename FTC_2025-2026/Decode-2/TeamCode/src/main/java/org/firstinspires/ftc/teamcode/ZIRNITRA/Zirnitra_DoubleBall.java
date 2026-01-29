package org.firstinspires.ftc.teamcode.ZIRNITRA;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zirnitra_DoubleBall", group = "Zirnitra")
public class Zirnitra_DoubleBall extends LinearOpMode {

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
    private static final double LOW_RPM = 3750;
    private static final double HIGH_RPM = 4400.0;
    private static final double RPM_TOLERANCE = 150.0;

    // 發射角度參數
    private static final double SHOOTERANGLE_INIT_POSITION = 0.68;

    // 在狀態變數區域添加
    private int rpmStableCounter = 0;
    private static final int RPM_STABLE_THRESHOLD = 3;  // 需連續 3 次達標

    // Trigger 參數
    private static final double TRIGGER_INIT_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.235;
    private static final double TRIGGER_THRESHOLD = 0.2;

    // 進球機構參數
    private static final double INTAKE_POWER = 0.5;

    // intake_2 (middle) 參數
    private static final int TOTAL_BALLS = 2;  // 射2顆球後停止

    // 發射器狀態
    private boolean shooterOn = false;
    private boolean feedEnabled = false;
    private boolean isHighVelocityMode = true;
    private double targetRPM = 0;
    private double currentShooterAngle = SHOOTERANGLE_INIT_POSITION;
    private int ballCount = 0;
    private boolean isFiring = false;

    // 按鍵邊緣檢測
    private boolean prevX = false;
    private boolean prevDpadLeft = false;
    private boolean prevBack = false;
    private boolean prevTrigger = false;

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
            handleDriveControls();
            handleShooterControls();
            handleTriggerControls();
            handleIntakeControls();

            if(gamepad1.left_bumper){
                Trigger.setPosition(TRIGGER_FIRE_POSITION);
            }

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

        shooterAngle_Right = hardwareMap.get(Servo.class, "shooterAngle_Right");
        shooterAngle_Left = hardwareMap.get(Servo.class, "shooterAngle_Left");
        shooterAngle_Right.setDirection(Servo.Direction.FORWARD);
        shooterAngle_Left.setDirection(Servo.Direction.REVERSE);
        shooterAngle_Right.setPosition(SHOOTERANGLE_INIT_POSITION);
        shooterAngle_Left.setPosition(SHOOTERANGLE_INIT_POSITION);
    }
    private void initializeIntakeMotors() {
        intake_1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");
        intake_2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // 底盤控制
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

    private void handleShooterControls() {
        boolean xPressed = gamepad1.x && !prevX;
        boolean dpadLeftPressed = gamepad1.dpad_left && !prevDpadLeft;
        boolean backPressed = gamepad1.right_bumper && !prevBack;

        if (xPressed) {
            shooterOn = true;
            isHighVelocityMode = true;
            intake_1.setPower(0);
            targetRPM = HIGH_RPM;
            ballCount = 0;  // 重置球數
            isFiring = false;
            intake_2.setPower(0.8);  // 啟動 intake_2
        }

        if (dpadLeftPressed) {
            shooterOn = true;
            isHighVelocityMode = false;
            intake_1.setPower(0);
            targetRPM = LOW_RPM;
            intake_2.setPower(0.8);  // 啟動 intake_2
        }

        if (backPressed) {
            shooterOn = false;
            feedEnabled = false;
            targetRPM = 0;
            ballCount = 0;
            isFiring = false;
            intake_2.setPower(0);  // 停止 intake_2
        }

        prevX = gamepad1.x;
        prevDpadLeft = gamepad1.dpad_left;
        prevBack = gamepad1.right_bumper;

        updateShooterVelocity();
    }

    /**
     * 處理觸發器控制
     * Right Trigger：發射（需發射器達到目標轉速）
     */
    private void handleTriggerControls() {
        boolean triggerPressed = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean triggerReleased = !triggerPressed && prevTrigger;  // 放開瞬間

        if (feedEnabled && triggerReleased && ballCount < TOTAL_BALLS && !isFiring) {
            ballCount++;
            isFiring = true;

            // 射完指定數量的球後停止 shooter
            if (ballCount >= TOTAL_BALLS) {
                shooterOn = false;
                feedEnabled = false;
                targetRPM = 0;
                rpmStableCounter = 0;
                intake_2.setPower(0);
            }
        }

        // 按住 Trigger 時停止 intake_2，發射
        boolean canFire = feedEnabled && triggerPressed;
        if (canFire) {
            intake_2.setPower(0);  // 按住時停止 intake_2
        }
        Trigger.setPosition(canFire ? TRIGGER_FIRE_POSITION : TRIGGER_INIT_POSITION);

        prevTrigger = triggerPressed;
    }

    // 新發射器速度並檢查是否達到目標速度
    private void updateShooterVelocity() {
        if (shooterOn) {
            double targetTicks = (targetRPM * SHOOTER_TICKS_PER_REV) / 60.0;

            shooter_Left.setVelocity(targetTicks);
            shooter_Right.setVelocity(targetTicks);

            double leftRPM = (shooter_Left.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0;
            double rightRPM = Math.abs((shooter_Right.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0);

            boolean inTolerance = Math.abs(leftRPM - targetRPM) <= RPM_TOLERANCE &&
                    Math.abs(rightRPM - targetRPM) <= RPM_TOLERANCE;

            if (inTolerance) {
                rpmStableCounter++;
            } else {
                rpmStableCounter = 0;
            }

            feedEnabled = rpmStableCounter >= RPM_STABLE_THRESHOLD;
        } else {
            shooter_Left.setVelocity(0);
            shooter_Right.setVelocity(0);
            feedEnabled = false;
            rpmStableCounter = 0;
        }
    }

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

    // 遙測與工具 (Telemetry & Utilities)
    private void updateTelemetry() {
        double leftRPM = Math.abs((shooter_Left.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0);
        double rightRPM = Math.abs((shooter_Right.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0);
        double avgRPM = (leftRPM + rightRPM) / 2.0;
        double error = avgRPM - targetRPM;

        telemetry.addLine("══════ 系統狀態 ══════");
        telemetry.addData("模式", isHighVelocityMode ? "遠距離 (HIGH)" : "近距離 (LOW)");
        telemetry.addData("可發射", feedEnabled ? "✓ YES" : "✗ NO");
        telemetry.addData("已射球數", "%d/%d", ballCount, TOTAL_BALLS);

        telemetry.addLine("══════ 速度資訊 ══════");
        telemetry.addData("目標 RPM", "%.2f", targetRPM);
        telemetry.addData("左馬達", "%.2f RPM", leftRPM);
        telemetry.addData("右馬達", "%.2f RPM", rightRPM);
        telemetry.addData("Error", "%+.1f RPM", error);
        telemetry.addData("轉速達標次數", rpmStableCounter);

        telemetry.addLine("══════ Servo 狀態 ══════");
        telemetry.addData("Trigger", "%.3f", Trigger.getPosition());
        telemetry.addData("ShooterAngle", "%.3f", currentShooterAngle);

        telemetry.addLine("══════ intake_2 狀態 ══════");
        telemetry.addData("Power", "%.2f", intake_2.getPower());
        telemetry.addData("位置", intake_2.getCurrentPosition());

        telemetry.addLine("══════ 操作說明 ══════");
        telemetry.addData("發射器", "X=遠, DpadLeft=近, RB=停");

        telemetry.update();
    }
}

