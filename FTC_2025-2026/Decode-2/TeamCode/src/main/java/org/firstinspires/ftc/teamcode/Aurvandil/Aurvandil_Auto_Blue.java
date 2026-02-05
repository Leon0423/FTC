package org.firstinspires.ftc.teamcode.Aurvandil;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Aurvandil_Auto_Blue", group = "Aurvandil")
public class Aurvandil_Auto_Blue extends LinearOpMode {

    // ===== 底盤馬達 =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== 射球機構 =====
    private DcMotorEx shooterMotor;
    private DcMotor intake1, intake2;

    // ===== 編碼器參數 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 3200;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // 1. 啟動 Shooter (預熱)
            setShooterRPM(TARGET_RPM);

            // 2. 直走 1.7 秒
            driveForward(-0.5, 1700);

            // 3. 等待 Shooter 達到目標轉速
            sleep(500);

            // 4. 射球 (啟動 intake2 作為 feeder) 3 秒
            shoot(3000);

            // 5. 停止 Shooter
            shooterMotor.setVelocity(0);

            // 6. 側移 1.5 秒
            driveStrafe(-0.5, 1500);

            // 7. 停止所有馬達
            stopAllMotors();
        }
    }

    private void initializeHardware() {
        // 底盤馬達初始化
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // 左側馬達反轉
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // 設定煞車模式
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // 射球機構初始化
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void setShooterRPM(double rpm) {
        double ticksPerSecond = rpm * SHOOTER_TICKS_PER_REV / 60.0;
        shooterMotor.setVelocity(ticksPerSecond);
    }

    private void driveForward(double power, long durationMs) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(durationMs);
        stopDriveMotors();
    }

    private void driveStrafe(double power, long durationMs) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(durationMs);
        stopDriveMotors();
    }

    private void shoot(long durationMs) {
        intake2.setPower(1);
        sleep(durationMs);
        intake2.setPower(0);
    }

    private void stopDriveMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void stopAllMotors() {
        stopDriveMotors();
        shooterMotor.setVelocity(0);
        intake1.setPower(0);
        intake2.setPower(0);
    }
}
