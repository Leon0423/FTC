package org.firstinspires.ftc.teamcode.CMS.ZIRNITRA;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zirnitra_TeleOpMode")
public class Zirnitra_TeleOpMode extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════
    // Hardware Declarations (硬體宣告)
    // ═══════════════════════════════════════════════════════════════
    private DcMotor FR, BR, FL, BL;
    private DcMotor intake_1, intake_2;
    private DcMotorEx shooter_Right, shooter_Left;
    private Servo Trigger;

    // ═══════════════════════════════════════════════════════════════
    // Constants (常數設定)
    // ═══════════════════════════════════════════════════════════════
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double TRIGGER_INIT_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.235;

    private static final double LOW_VELOCITY_RPM = 3750.0;
    private static final double HIGH_VELOCITY_RPM = 4800.0;
    private static final double RPM_TOLERANCE = 80.0;
    private static final double VELOCITY_HYSTERESIS_FACTOR = 1.2;

    private static final double INTAKE_POWER = 1.0;
    private static final double INTAKE_2_POWER = 0.3;
    private static final double TRIGGER_THRESHOLD = 0.2;
    private static final double DRIVE_SCALE = 0.8;

    private static final int RPM_STABLE_COUNT_REQUIRED = 3;

    // ═══════════════════════════════════════════════════════════════
    // State Variables (狀態變數)
    // ═══════════════════════════════════════════════════════════════
    private boolean shooterOn = false;
    private boolean feedEnabled = false;
    private boolean isHighVelocityMode = true;
    private int rpmStableCounter = 0;
    private boolean wasVelocityReached = false;

    // 快取計算值
    private double targetTicks = 0;
    private double toleranceTicks = 0;

    // Button Edge Detection
    private boolean prevX, prevY, prevBack;

    // ═══════════════════════════════════════════════════════════════
    // Main Program (主程式)
    // ═══════════════════════════════════════════════════════════════
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        showInitTelemetry();
        waitForStart();

        while (opModeIsActive()) {
            handleDriveControls();
            handleShooterControls();
            handleTriggerControls();
            handleIntakeControls();
            updateTelemetry();
        }

        stopAllMotors();
    }

    // ═══════════════════════════════════════════════════════════════
    // Hardware Initialization (硬體初始化)
    // ═══════════════════════════════════════════════════════════════
    private void initializeHardware() {
        initializeDriveMotors();
        initializeIntakeMotors();
        initializeShooterMotors();
        initializeServos();
    }

    private void initializeDriveMotors() {
        DcMotor[] driveMotors = {
                FR = hardwareMap.get(DcMotor.class, "FR"),
                FL = hardwareMap.get(DcMotor.class, "FL"),
                BR = hardwareMap.get(DcMotor.class, "BR"),
                BL = hardwareMap.get(DcMotor.class, "BL")
        };

        for (DcMotor motor : driveMotors) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
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

        for (DcMotorEx shooter : new DcMotorEx[]{shooter_Right, shooter_Left}) {
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void initializeServos() {
        Trigger = hardwareMap.get(Servo.class, "Trigger");
        Trigger.setDirection(Servo.Direction.REVERSE);
        Trigger.setPosition(TRIGGER_INIT_POSITION);
    }

    // ═══════════════════════════════════════════════════════════════
    // Drive Controls (底盤控制)
    // ═══════════════════════════════════════════════════════════════
    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x + gamepad2.right_stick_x * 0.8;

        double fr = forward - rotate - strafe;
        double fl = forward + rotate + strafe;
        double br = forward - rotate + strafe;
        double bl = forward + rotate - strafe;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fr), Math.abs(fl)),
                Math.max(Math.abs(br), Math.abs(bl))));

        FR.setPower(fr / max * DRIVE_SCALE);
        FL.setPower(fl / max * DRIVE_SCALE);
        BR.setPower(br / max * DRIVE_SCALE);
        BL.setPower(bl / max * DRIVE_SCALE);
    }

    // ═══════════════════════════════════════════════════════════════
    // Shooter Controls (發射器控制)
    // ═══════════════════════════════════════════════════════════════
    private void handleShooterControls() {
        boolean xPressed = gamepad1.x && !prevX;
        boolean yPressed = gamepad1.y && !prevY;
        boolean stopPressed = (gamepad1.right_bumper || gamepad2.right_bumper) && !prevBack;

        if (xPressed) activateShooter(true);
        if (yPressed) activateShooter(false);
        if (stopPressed) deactivateShooter();

        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevBack = gamepad1.right_bumper || gamepad2.right_bumper;

        updateShooterVelocity();
    }

    private void activateShooter(boolean highVelocity) {
        shooterOn = true;
        isHighVelocityMode = highVelocity;
        rpmStableCounter = 0;
        wasVelocityReached = false;
        intake_1.setPower(0);

        // 預先計算目標值
        double targetRPM = highVelocity ? HIGH_VELOCITY_RPM : LOW_VELOCITY_RPM;
        targetTicks = rpmToTicks(targetRPM);
        toleranceTicks = rpmToTicks(RPM_TOLERANCE);
    }

    private void deactivateShooter() {
        shooterOn = false;
        feedEnabled = false;
        rpmStableCounter = 0;
        wasVelocityReached = false;
    }

    private double rpmToTicks(double rpm) {
        return rpm * SHOOTER_TICKS_PER_REV / 60.0;
    }

    private void updateShooterVelocity() {
        if (!shooterOn) {
            shooter_Left.setVelocity(0);
            shooter_Right.setVelocity(0);
            feedEnabled = false;
            return;
        }

        shooter_Left.setVelocity(targetTicks);
        shooter_Right.setVelocity(targetTicks);

        double leftError = Math.abs(Math.abs(shooter_Left.getVelocity()) - targetTicks);
        double rightError = Math.abs(Math.abs(shooter_Right.getVelocity()) - targetTicks);
        double effectiveTolerance = wasVelocityReached ?
                toleranceTicks * VELOCITY_HYSTERESIS_FACTOR : toleranceTicks;

        boolean inRange = leftError <= effectiveTolerance && rightError <= effectiveTolerance;

        if (inRange) {
            if (++rpmStableCounter >= RPM_STABLE_COUNT_REQUIRED) {
                feedEnabled = true;
                wasVelocityReached = true;
            }
        } else {
            rpmStableCounter = 0;
            feedEnabled = false;
            if (leftError > toleranceTicks * VELOCITY_HYSTERESIS_FACTOR ||
                    rightError > toleranceTicks * VELOCITY_HYSTERESIS_FACTOR) {
                wasVelocityReached = false;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // Trigger & Intake Controls
    // ═══════════════════════════════════════════════════════════════
    private void handleTriggerControls() {
        boolean canFire = feedEnabled && gamepad1.right_trigger > TRIGGER_THRESHOLD;
        Trigger.setPosition(canFire ? TRIGGER_FIRE_POSITION : TRIGGER_INIT_POSITION);
    }

    private void handleIntakeControls() {
        double power = 0, power2 = 0;

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            power = INTAKE_POWER;
            power2 = INTAKE_2_POWER;
        } else if (gamepad1.b || gamepad2.b) {
            power = -INTAKE_POWER;
            power2 = -INTAKE_2_POWER;
        }

        intake_1.setPower(power);
        intake_2.setPower(power2);

        if (power != 0) rpmStableCounter = 0;
    }

    // ═══════════════════════════════════════════════════════════════
    // Telemetry
    // ═══════════════════════════════════════════════════════════════
    private void showInitTelemetry() {
        telemetry.addData("Status", "Ready");
        telemetry.addLine(
                "【Gamepad1 - 主要控制】");
        telemetry.addData("X", "啟動遠距離");
        telemetry.addData("Y", "啟動近距離");
        telemetry.addData("LB", "吸球");
        telemetry.addData("B", "吐球");
        telemetry.addData("RB", "緊急停止");
        telemetry.addData("RT", "發射 (需達目標速度)");

        telemetry.addLine();
        telemetry.addLine("【Gamepad2 - 輔助控制】");
        telemetry.addData("LB", "吸球");
        telemetry.addData("B", "吐球");
        telemetry.addData("RB", "緊急停止");
        telemetry.update();
    }

    private void updateTelemetry() {
        double targetRPM = shooterOn ? (isHighVelocityMode ? HIGH_VELOCITY_RPM : LOW_VELOCITY_RPM) : 0;
        double leftRPM = shooter_Left.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV;
        double rightRPM = shooter_Right.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV;

        telemetry.addData("Shooter", "%s | %s | Fire: %s",
                shooterOn ? "ON" : "OFF",
                isHighVelocityMode ? "FAR" : "NEAR",
                feedEnabled ? "✓" : "✗");
        telemetry.addData("Target", "%.0f RPM", targetRPM);
        telemetry.addData("L/R RPM", "%.0f / %.0f", leftRPM, rightRPM);
        telemetry.addData("Stable", "%d/%d", rpmStableCounter, RPM_STABLE_COUNT_REQUIRED);
        telemetry.update();
    }

    private void stopAllMotors() {
        FL.setPower(0); FR.setPower(0); BL.setPower(0); BR.setPower(0);
        shooter_Left.setVelocity(0); shooter_Right.setVelocity(0);
        intake_1.setPower(0); intake_2.setPower(0);
    }
}
