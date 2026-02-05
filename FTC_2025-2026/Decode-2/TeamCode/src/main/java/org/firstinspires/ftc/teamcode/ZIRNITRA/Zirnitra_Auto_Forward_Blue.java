package org.firstinspires.ftc.teamcode.ZIRNITRA;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Zirnitra_Auto_Forward_Blue")
public class Zirnitra_Auto_Forward_Blue extends LinearOpMode {

    // Hardware
    private DcMotor FR, BR, FL, BL;
    private DcMotor intake_1, intake_2;
    private DcMotorEx shooter_Right, shooter_Left;
    private Servo Trigger;

    // Constants
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double TRIGGER_INIT_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.235;
    private static final double HIGH_VELOCITY_RPM = 4800.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // 1. 啟動 Shooter (預熱)
            setShooterRPM(HIGH_VELOCITY_RPM);

            driveForward(0.5, 200);

            driveRotate(-0.5, 250);

            // 3. 等待 Shooter 達到目標轉速
            sleep(1000);

            // 4. 射球
            intake_2.setPower(1.0);
            intake_1.setPower(1.0);
            shoot(4000);
            Trigger.setPosition(TRIGGER_INIT_POSITION);
            sleep(500);
            Trigger.setPosition(TRIGGER_FIRE_POSITION);

            driveRotate(0.5, 300);

            driveStrafe(-0.5, 850);

            // 5. 停止 Shooter
            intake_2.setPower(0);
            shooter_Left.setVelocity(0);
            shooter_Right.setVelocity(0);

            // 7. 停止所有馬達
            stopAllMotors();
        }
    }

    private void initializeHardware() {
        // Drive motors
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        for (DcMotor motor : new DcMotor[]{FR, FL, BR, BL}) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Shooter motors
        shooter_Right = hardwareMap.get(DcMotorEx.class, "shooter_Right");
        shooter_Left = hardwareMap.get(DcMotorEx.class, "shooter_Left");

        for (DcMotorEx shooter : new DcMotorEx[]{shooter_Right, shooter_Left}) {
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Trigger servo
        Trigger = hardwareMap.get(Servo.class, "Trigger");
        Trigger.setDirection(Servo.Direction.REVERSE);
        Trigger.setPosition(TRIGGER_INIT_POSITION);

        intake_1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");

        intake_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void setShooterRPM(double rpm) {
        double ticksPerSecond = rpm * SHOOTER_TICKS_PER_REV / 60.0;
        shooter_Left.setVelocity(ticksPerSecond);
        shooter_Right.setVelocity(ticksPerSecond);
    }

    private void driveForward(double power, long durationMs) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
        sleep(durationMs);stopDriveMotors();
    }

    private void driveRotate(double power, long durantionMs){
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(power);
        BR.setPower(-power);
        sleep(durantionMs);stopDriveMotors();
    }

    private void driveStrafe(double power, long durationMs) {
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(-power);
        BR.setPower(power);
        sleep(durationMs);
        stopDriveMotors();
    }

    private void shoot(long durationMs) {
        Trigger.setPosition(TRIGGER_INIT_POSITION);
        sleep(durationMs);
        Trigger.setPosition(TRIGGER_FIRE_POSITION);
    }

    private void stopDriveMotors() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void stopAllMotors() {
        stopDriveMotors();
        shooter_Left.setVelocity(0);
        shooter_Right.setVelocity(0);
    }
}
