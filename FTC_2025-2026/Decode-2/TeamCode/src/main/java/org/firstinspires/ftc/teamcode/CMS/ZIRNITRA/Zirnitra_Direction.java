package org.firstinspires.ftc.teamcode.CMS.ZIRNITRA;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Zirnitra_Direction extends LinearOpMode {

    // 底盤馬達 (Drive Motors)
    private DcMotor FR, BR, FL, BL;

    // 進球機構 (Intake Motors)
    private DcMotor intake_1, intake_2;

    // 發射器馬達 (Shooter Motors)
    private DcMotorEx shooter_Right, shooter_Left;

    // 伺服馬達 (Servos)
    private Servo shooterAngle_Right, shooterAngle_Left, Trigger;

    // ShooterAngle Servo 參數
    private static final double SHOOTERANGLE_INIT_POSITION = 0.0;
    private static final double SHOOTERANGLE_MIN_LIMIT = 0.0;
    private static final double SHOOTERANGLE_MAX_LIMIT = 0.16;
    private static final double SHOOTERANGLE_STEP = 0.01;

    // Trigger Servo 參數
    private static final double TRIGGER_INIT_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.235;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();
        while (opModeIsActive()){
            // 測試底盤馬達
            if (gamepad1.dpad_up) {
                FR.setPower(0.5);
                telemetry.addLine("Testing FR Motor");
            } else {
                FR.setPower(0);
            }

            if (gamepad1.dpad_down) {
                BR.setPower(0.5);
                telemetry.addLine("Testing BR Motor");
            } else {
                BR.setPower(0);
            }

            if (gamepad1.dpad_left) {
                FL.setPower(0.5);
                telemetry.addLine("Testing FL Motor");
            } else {
                FL.setPower(0);
            }

            if (gamepad1.dpad_right) {
                BL.setPower(0.5);
                telemetry.addLine("Testing BL Motor");
            } else {
                BL.setPower(0);
            }

            // 測試進球機構馬達
            if (gamepad1.left_bumper) {
                intake_1.setPower(0.5);
                telemetry.addLine("Testing intake_1 Motor");
            } else {
                intake_1.setPower(0);
            }

            if (gamepad1.right_bumper) {
                intake_2.setPower(0.5); // reverse
                telemetry.addLine("Testing intake_2 Motor");
            } else {
                intake_2.setPower(0);
            }

            // 測試發射器馬達
            if (gamepad1.left_trigger > 0.1) {
                shooter_Left.setPower(0.5);
                telemetry.addLine("Testing shooter_Left Motor");
            } else {
                shooter_Left.setPower(0);
            }

            if (gamepad1.right_trigger > 0.1) {
                shooter_Right.setPower(0.5);
                telemetry.addLine("Testing shooter_Right Motor");
            } else {
                shooter_Right.setPower(0);
            }

            // 測試 ShooterAngle Servos
            if (gamepad1.aWasPressed()) {
                double newPos = shooterAngle_Right.getPosition() + SHOOTERANGLE_STEP;
                shooterAngle_Right.setPosition(newPos);
                shooterAngle_Left.setPosition(newPos);
                telemetry.addLine("ShooterAngle Position: " + String.format("%.3f", newPos));
            }
            if (gamepad1.yWasPressed()) {
                double newPos = shooterAngle_Right.getPosition() - SHOOTERANGLE_STEP;
                shooterAngle_Right.setPosition(newPos);
                shooterAngle_Left.setPosition(newPos);
                telemetry.addLine("ShooterAngle Position: " + String.format("%.3f", newPos));
            }
            if (gamepad1.xWasPressed()) {
                shooterAngle_Right.setPosition(SHOOTERANGLE_INIT_POSITION);
                shooterAngle_Left.setPosition(SHOOTERANGLE_INIT_POSITION);
                telemetry.addLine("ShooterAngle Reset to Init");
            }

            if(gamepad1.b){
                Trigger.setPosition(TRIGGER_FIRE_POSITION);
            } else {
                Trigger.setPosition(TRIGGER_INIT_POSITION);
            }

            // 顯示當前 Servo 位置
            telemetry.addLine("\n=== Servo Positions ===");
            telemetry.addData("ShooterAngle", "%.3f", shooterAngle_Right.getPosition());

            telemetry.update();
        }

    }

    private void initializeHardware() {
        initializeDriveMotors();
        initializeIntakeMotors();
        initializeShooterMotors();
        initializeServos();

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
        intake_2.setDirection(DcMotorSimple.Direction.FORWARD);

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

}
