package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zirnitra_TeleOpMode")
public class Zirnitra extends LinearOpMode {

    // ===== Drive Motors =====
    DcMotor FR, BR, FL, BL;

    // ===== Intake =====
    DcMotor intake_1, intake_2;

    // ===== Shooter =====
    DcMotorEx shooter_Right, shooter_Left;

    // ===== Servos =====
    Servo shooterAngle_Right, shooterAngle_Left, Trigger = null;

    double target_pos = 0;

    // ===== 參數 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double TRIGGER_INIT_POSITION = 0;
    private static final double TRIGGER_TARGET_POSITION = 0.235;
    private static final double SHOOTERANGLE_INIT_POSITION = 0.16;
    private static final double SHOOTERANGLE_MAX_LIMIT = 0;
    private static final double SHOOTERANGLE_TARGET_POSITION = Math.min(SHOOTERANGLE_INIT_POSITION, SHOOTERANGLE_MAX_LIMIT);
    private static final double SHOOTERANGLE_HIGH_POSITION = 0.05;
    private static final double SHOOTERANGLE_LOW_POSITION = 0.11;


    // ===== PIDF =====
    /*private static final double SHOOTER_P = 230.0;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 11.5;
     */

    // ===== 速度設定 =====
    private static final double LOW_VELOCITY = 1000;
    private static final double HIGH_VELOCITY = 2000;

    // ===== 速度容差 =====
    private static final double HIGH_VELOCITY_TOLERANCE = 40;
    private static final double LOW_VELOCITY_TOLERANCE = 40;

    // ===== Servo/Motor 功率設定 =====
    private static final double INTAKE_POWER = 0.8;

    // ===== 機構狀態 =====
    private boolean shooterOn = false;
    private boolean feedEnabled = false;

    // ===== 按鍵邊緣檢測 =====
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevDpadLeft = false;
    private boolean isHighVelocityMode = true;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDriveControls();
            handleTriggerControls();
            handleShooterControls();
            handleIntakeControls();
            updateTelemetry();
            idle();
        }

        stopAllMotors();
    }

    private void initializeHardware() {
        // 底盤馬達
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

        // Intake
        intake_1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");
        intake_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Shooter
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

        // 設定 PIDF 參數
        /*shooter_Right.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter_Left.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
         */

        // Trigger Servo
        Trigger = hardwareMap.get(Servo.class, "Trigger");
        Trigger.setDirection(Servo.Direction.REVERSE);
        Trigger.setPosition(TRIGGER_INIT_POSITION);

        // ShooterAngle Servo
        shooterAngle_Right = hardwareMap.get(Servo.class, "shooterAngle_Right");
        shooterAngle_Left = hardwareMap.get(Servo.class, "shooterAngle_Left");
        shooterAngle_Right.setDirection(Servo.Direction.FORWARD);
        shooterAngle_Left.setDirection(Servo.Direction.REVERSE);
        shooterAngle_Right.setPosition(SHOOTERANGLE_INIT_POSITION);
        shooterAngle_Left.setPosition(SHOOTERANGLE_INIT_POSITION);

        stopAllMotors();
    }

    private void handleDriveControls() {
        double y = gamepad1.left_stick_y;
        double rx = gamepad1.left_stick_x;
        double x = gamepad1.right_stick_x;

        FL.setPower(-y + x + rx);
        FR.setPower(-y - x - rx);
        BL.setPower(y - x + rx);
        BR.setPower(y + x - rx);
    }

    private void handleTriggerControls() {
        // 只有當 shooter 達到目標轉速時，trigger 才會動作
        if (feedEnabled && gamepad1.right_trigger > 0.2) {
            target_pos = TRIGGER_TARGET_POSITION;
        } else {
            target_pos = TRIGGER_INIT_POSITION;
        }
        Trigger.setPosition(target_pos);
    }

    private void handleShooterControls() {
        boolean xNow = gamepad1.x;
        boolean backNow = gamepad1.right_bumper;
        boolean dpadLeftNow = gamepad1.dpad_left;

        if (xNow && !prevX) {
            shooterOn = true;
            isHighVelocityMode = true;
            // 遠距離模式：設定高角度
            shooterAngle_Right.setPosition(SHOOTERANGLE_HIGH_POSITION);
            shooterAngle_Left.setPosition(SHOOTERANGLE_HIGH_POSITION);
        }

        if (dpadLeftNow && !prevDpadLeft) {
            shooterOn = true;
            isHighVelocityMode = false;
            // 近距離模式：設定低角度（初始位置）
            shooterAngle_Right.setPosition(SHOOTERANGLE_LOW_POSITION);
            shooterAngle_Left.setPosition(SHOOTERANGLE_LOW_POSITION);
        }

        if (backNow && !prevBack) {
            shooterOn = false;
            feedEnabled = false;
            shooter_Left.setVelocity(0);
            shooter_Right.setVelocity(0);
            // 關閉時回到初始角度
            shooterAngle_Right.setPosition(SHOOTERANGLE_INIT_POSITION);
            shooterAngle_Left.setPosition(SHOOTERANGLE_INIT_POSITION);
        }

        prevX = xNow;
        prevBack = backNow;
        prevDpadLeft = dpadLeftNow;

        if (shooterOn) {
            double velocity = isHighVelocityMode ? HIGH_VELOCITY : LOW_VELOCITY;
            double tolerance = isHighVelocityMode ? HIGH_VELOCITY_TOLERANCE : LOW_VELOCITY_TOLERANCE;
            shooter_Left.setVelocity(velocity);
            shooter_Right.setVelocity(velocity);

            // 檢查是否達到目標速度
            double leftVelocity = shooter_Left.getVelocity();
            double rightVelocity = shooter_Right.getVelocity();
            feedEnabled = (Math.abs(leftVelocity - velocity) <= tolerance) &&
                          (Math.abs(rightVelocity - velocity) <= tolerance);
        } else {
            shooter_Left.setVelocity(0);
            shooter_Right.setVelocity(0);
            feedEnabled = false;
        }
    }

    private void handleIntakeControls() {
        if (gamepad1.a) {
            intake_1.setPower(INTAKE_POWER);
            intake_2.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intake_1.setPower(0);
            intake_2.setPower(0);
        }
    }

    private double calculateRPM(double ticksPerSecond) {
        // 移除 shooterOn 檢查，讓 RPM 能正確顯示減速過程
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    private void updateTelemetry() {
        double leftVelocity = shooter_Left.getVelocity();
        double rightVelocity = shooter_Right.getVelocity();
        double targetVelocity = isHighVelocityMode ? HIGH_VELOCITY : LOW_VELOCITY;
        double rpm_left = calculateRPM(leftVelocity);
        double rpm_right = calculateRPM(rightVelocity);

        telemetry.addLine("══════ 系統狀態 ══════");
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("模式", isHighVelocityMode ? "遠距離" : "近距離");
        telemetry.addLine("══════ 速度資訊 ══════");
        telemetry.addData("目標速度", String.format("%.0f ticks/s", shooterOn ? targetVelocity : 0.0));
        telemetry.addData("左馬達速度", String.format("%.1f ticks/s", leftVelocity));
        telemetry.addData("右馬達速度", String.format("%.1f ticks/s", rightVelocity));
        telemetry.addData("左邊 RPM", String.format("%.0f", rpm_left));
        telemetry.addData("右邊 RPM", String.format("%.0f", rpm_right));
        telemetry.addData("達到目標速度", feedEnabled ? "YES" : "NO");
        telemetry.addData("Trigger Position", Trigger.getPosition());
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
