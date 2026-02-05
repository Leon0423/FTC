package org.firstinspires.ftc.teamcode.Aurvandil;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Aurvandil_absolute", group = "Aurvandil")
public class Aurvandil_absolute extends LinearOpMode {

    // ===== 底盤馬達 =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== 射球機構 =====
    private DcMotorEx shooterMotor;      // goBILDA 5202-0002-0001 (28 CPR)
    private DcMotor intake1, intake2;

    IMU imu;

    // ===== 編碼器參數 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // 每圈 28 個 ticks

    // ===== 預設速度設定 =====
    private static final double LOw_RPM = 3500;  // 遠距離射球速度


    // ===== RPM 微調參數 =====
    private double targetRPM = 0;                 // 當前目標 RPM（可動態調整）

    // ===== 速度容差（依模式自動切換）=====
    private static final double HIGH_VELOCITY_TOLERANCE = 150;  // 高速模式容差

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();

        // ===== 主循環 =====
        while (opModeIsActive()) {
            handleDriveControls();        // 底盤控制

            if (gamepad1.a || gamepad1.left_bumper || gamepad2.a || gamepad2.left_bumper) {
                intake1.setPower(1);
            } else if(gamepad1.b || gamepad2.b){
                intake1.setPower(-1);
            }else{
                intake1.setPower(0);
            }

            if (gamepad1.x || gamepad2.x){
                shooterMotor.setVelocity(caculateTargetVelocity(LOw_RPM));
            }

            if (gamepad1.y || gamepad2.y) {
                if((shooterMotor.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0 >  targetRPM - HIGH_VELOCITY_TOLERANCE){
                    intake2.setPower(1);
                } else {
                    intake2.setPower(0);
                }
            }


            if((gamepad1.right_trigger > 0.5) || gamepad2.right_trigger > 0.5){
                if((shooterMotor.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0 >  targetRPM - HIGH_VELOCITY_TOLERANCE){
                    intake2.setPower(1);
                } else {
                    intake2.setPower(0);
                }
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper){
                stopAllMotors();
            }

            telemetry.addData("RPM", shooterMotor.getVelocity() * 60/ 28);
            telemetry.update();
            idle();

        }

        stopAllMotors();
    }

    private void initializeHardware() {
        // 底盤馬達初始化
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // 右側馬達反轉（統一前進方向）
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // 設定煞車模式（停止時鎖定）
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 底盤馬達不使用編碼器（直接功率控制）
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        // 射球與進球機構初始化；
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


        stopAllMotors();
    }

    // 處理底盤控制（麥克納姆輪全向移動）
    private void handleDriveControls() {
        double y = -gamepad1.left_stick_y * 0.9; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 0.9;
        double rx = gamepad1.right_stick_x * 0.9 + gamepad2.right_stick_x * 0.7;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.start) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    private double caculateTargetVelocity(double RPM) {
        return RPM * (28 / 60.0);
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        shooterMotor.setVelocity(0);
        intake1.setPower(0);
        intake2.setPower(0);
    }
}
