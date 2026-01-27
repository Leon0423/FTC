package org.firstinspires.ftc.teamcode.TEAM_1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "team_1_AdjustMotor", group = "TeleOp")
public class final_test extends LinearOpMode {

    // ===== 底盤馬達 =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== 射球機構 =====
    private DcMotorEx shooterMotor;      // goBILDA 5202-0002-0001 (28 CPR)
    private CRServo feederServo;         // 連續旋轉伺服馬達

    // ===== 吸球機構 =====
    private DcMotor intakeMotor;         // 312 RPM 吸球馬達

    // ===== 編碼器參數 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // 每圈 28 個 ticks

    // ===== PIDF 係數 =====
    private static final double SHOOTER_P = 19;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 12.0334;

    // ===== 預設速度設定 =====
    private static final double LOW_RPM = 3720;   // 近距離射球速度
    private static final double HIGH_RPM = 4757.14;  // 遠距離射球速度

    // ===== RPM 微調參數 =====
    private double targetRPM = 0;                 // 當前目標 RPM（可動態調整）
    private static final double RPM_ADJUST_STEP = 50; // 每次微調步長

    // ===== 速度容差（依模式自動切換）=====
    private static final double HIGH_VELOCITY_TOLERANCE = 20;  // 高速模式容差
    private static final double LOW_VELOCITY_TOLERANCE = 20;   // 低速模式容差

    // ===== Servo 功率設定 =====
    private static final double FEEDER_OUTTAKE_POWER = 1.0;   // 吐球功率
    private static final double FEEDER_FEED_POWER = -1.0;     // 送球功率
    private static final double INTAKE_POWER = 0.5;           // 吸球功率

    // ===== 機構狀態旗標 =====
    private boolean shooterOn = false;           // Shooter 是否啟動
    private boolean feedEnabled = false;         // Feeder 是否在送球
    private boolean isHighVelocityMode = true;   // 當前速度模式（高速 true/低速 false）

    // ===== 按鍵邊緣檢測（防止連續觸發）=====
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevDpadLeft = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addData("Status", "✓ 初始化完成");
        telemetry.update();

        waitForStart();

        // ===== 主循環 =====
        while (opModeIsActive()) {
            handleDriveControls();        // 底盤控制

            if(gamepad1.dpad_up){
                frontRight.setPower(0.5);
            }
            if (gamepad1.dpad_down){
                frontLeft.setPower(0.5);//
            }
            if (gamepad1.dpad_right){//
                backRight.setPower(0.5);
            }
            if (gamepad1.dpad_left){
                backLeft.setPower(0.5);
            }
        }
    }

    /**
     * 初始化所有硬體設備
     */
    private void initializeHardware() {
        // 底盤馬達初始化
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // 右側馬達反轉（統一前進方向）
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

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
    }

    /**
     * 處理底盤控制（麥克納姆輪全向移動）
     */
    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;  // 前後
        double strafe = gamepad1.right_stick_x;   // 平移
        double rotate = gamepad1.left_stick_x;    // 旋轉
        double fr, fl, br, bl, scale;

        fr = forward - rotate - strafe;
        fl = forward + rotate + strafe;
        br = -forward - rotate + strafe;
        bl = -forward + rotate - strafe;

        scale = scaling_power(fr, fl, br, bl);

        frontRight.setPower(fr / scale);
        frontLeft.setPower(fl / scale);
        backRight.setPower(br / scale);
        backLeft.setPower(bl / scale);
    }

    private double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if (max <= 1) {
            max = 1;
        }
        return max;
    }
}