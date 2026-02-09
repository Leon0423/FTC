package org.firstinspires.ftc.teamcode.CMS.TEAM_1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "team_1_Auto_Blue", group = "Autonomous")
public class team_1_Auto_Blue extends LinearOpMode {

    // ===== 硬體宣告 =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;  // 底盤馬達
    private DcMotorEx shooterMotor;      // Shooter 馬達 (goBILDA 5202)
    private CRServo feederServo;         // Feeder 連續伺服馬達
    private DcMotor intakeMotor;         // Intake 馬達 (312 RPM)

    // ===== 射球速度設定 (RPM) =====
    private static final double LOW_RPM = 4000;   // 近距離目標轉速
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()){
            // 1. 啟動 Shooter (預熱)
            setShooterRPM(3500);

            intakeMotor.setPower(1);

            // 2. 直走 2 秒
            driveForward(-0.5, 1500);

            // 3. 等待 Shooter 達到目標轉速
            sleep(500);

            // 4. 射球 (啟動 Feeder)
            shoot(3000);

            // 5. 停止 Shooter
            shooterMotor.setPower(0);

            // 6. 後退 2 秒
            driveStrafe(-0.5, 1500);

            // 7. 停止所有馬達
            stopAllMotors();
        }


    }

    private void initializeHardware() {
        // --- 底盤馬達配置 ---
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // 左側馬達反轉 (確保正確的移動方向)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // 設定零功率煞車模式 (停止時鎖死)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 底盤不使用編碼器 (更快的響應)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Shooter 與 Feeder 配置 ---
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        feederServo = hardwareMap.get(CRServo.class, "feederServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Shooter 編碼器重置並啟用速度控制
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // 停止時自由滑行

        // 設定 PIDF 控制參數 (用於精確速度控制)
        // shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // Intake 設定
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // 設定 Shooter 目標 RPM
    private void setShooterRPM(double rpm) {
        double ticksPerRevolution = 28.0; // goBILDA 5202 編碼器解析度
        double ticksPerSecond = (rpm / 60.0) * ticksPerRevolution;
        shooterMotor.setVelocity(ticksPerSecond);
    }

    // 向前移動
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

    // 向後移動
    private void driveBackward(double power, long durationMs) {
        driveForward(-power, durationMs);
    }

    // 射球
    private void shoot(long durationMs) {
        feederServo.setPower(-1.0);  // 啟動 Feeder 推球
        sleep(durationMs);
        feederServo.setPower(0);    // 停止 Feeder
    }

    // 停止底盤馬達
    private void stopDriveMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // 停止所有馬達
    private void stopAllMotors() {
        stopDriveMotors();
        shooterMotor.setPower(0);
        feederServo.setPower(0);
        intakeMotor.setPower(0);
    }
}
