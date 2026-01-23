package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "team_1", group = "TeleOp")
public class team_1 extends OpMode {

    // ===== Drive =====
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== Shooter / Feeder =====
    DcMotor shooterMotor;      // goBILDA 5202-0002-0001 (28 CPR)
    CRServo feederServo;       // 連續旋轉 servo (CRServo)

    // ===== Intake =====
    DcMotor intakeMotor;       // 312 RPM intake

    // ===== RPM 計算 =====
    ElapsedTime rpmTimer = new ElapsedTime();
    int lastTicks = 0;

    // ===== 參數 =====
    static final double SHOOTER_POWER = 0.9;

    // goBILDA encoder：28 CPR（馬達端）
    static final double SHOOTER_TICKS_PER_REV = 7.0;

    // 送球門檻（可依實測調整）
    static final double FEED_ON_RPM  = 9000;
    static final double FEED_OFF_RPM = 10000;

    // ===== Servo 功率設定 =====
    // OUTTAKE：平常一直往外吐球（把球往外推走 / 防卡球）
    static final double FEEDER_OUTTAKE_POWER = 1.0; // 方向不對就改成 +0.35
    // FEED：按下 Y 才送球（往 shooter 方向送）
    static final double FEEDER_FEED_POWER    = -1.0;   // 方向不對就改成 -1.0

    static final double INTAKE_POWER = 0.5;

    // ===== 狀態 =====
    boolean shooterOn = false;     // X：啟動 shooter（維持）
    boolean feedEnabled = false;   // 內部 RPM 防抖狀態（只有按住 Y 才會用到）

    // ===== 按鍵 edge detect =====
    boolean prevX = false;
    boolean prevBack = false;

    @Override
    public void init() {

        frontLeft  = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft   = hardwareMap.dcMotor.get("BL");
        backRight  = hardwareMap.dcMotor.get("BR");

        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        feederServo  = hardwareMap.crservo.get("feederServo");
        intakeMotor  = hardwareMap.dcMotor.get("intakeMotor");

        // 左邊反轉
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ===== Shooter：開迴路，避免 RUN_USING_ENCODER 把轉速鎖住 =====
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // Intake 簡單控制
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastTicks = shooterMotor.getCurrentPosition();
        rpmTimer.reset();

        // 全部先停（servo 在 loop 會進入「平常吐球」狀態）
        shooterMotor.setPower(0);
        feederServo.setPower(0);
        intakeMotor.setPower(0);
    }

    @Override
    public void loop() {

        /* ========= 麥克納姆（最簡） ========= */
        double y  = gamepad1.left_stick_y;
        double rx  =  gamepad1.left_stick_x;
        double x =  gamepad1.right_stick_x;

        frontLeft.setPower(-y + x + rx);
        frontRight.setPower(-y - x - rx);
        backLeft.setPower(y - x + rx);
        backRight.setPower(y + x - rx);

        /* ========= 操作 =========
           X（按一下）：Shooter 開始旋轉（維持）
           Y（按住）：達 RPM 才送球；沒按住時 servo 一直吐球
           Back（按一下）：全部停止並重置（Shooter/Feeder）
        */
        boolean xNow = gamepad1.x;
        boolean backNow = gamepad1.right_bumper;

        // X：開 shooter（並重置 RPM 量測基準）
        if (xNow && !prevX) {
            shooterOn = true;
            lastTicks = shooterMotor.getCurrentPosition();
            rpmTimer.reset();
        }

        // Back：全停 + 重置
        if (backNow && !prevBack) {
            shooterOn = false;
            feedEnabled = false;

            shooterMotor.setPower(0);
            feederServo.setPower(0); // 先停一下，下一圈 loop 會回到吐球模式

            lastTicks = shooterMotor.getCurrentPosition();
            rpmTimer.reset();
        }

        prevX = xNow;
        prevBack = backNow;

        if (gamepad1.dpadLeftWasPressed()) {
            shooterMotor.setPower(0.4);
        }

        if (gamepad1.dpadUpWasPressed()) {
            feederServo.setPower(FEEDER_FEED_POWER);
        }
        if (gamepad1.dpadDownWasPressed()) {
            feederServo.setPower(0);
        }

        // Shooter 開關
        if (shooterOn) {
            shooterMotor.setPower(SHOOTER_POWER);
        } else {
            shooterMotor.setPower(0);
        }

        /* ========= RPM 計算 ========= */
        double rpm = 0.0;

        if (shooterOn) {
            double dt = rpmTimer.seconds();
            if (dt < 0.01) dt = 0.01;

            int currentTicks = shooterMotor.getCurrentPosition();
            int delta = currentTicks - lastTicks;

            rpm = (delta / SHOOTER_TICKS_PER_REV) / dt * 60.0; //
            lastTicks = currentTicks;
            rpmTimer.reset();
        } else {
            feedEnabled = false;
        }

        /* ========= Feeder Servo 邏輯（你要的版本） =========
           - 平常：一直向外吐球（OUTTAKE）
           - 按住 Y：只有在 shooterOn 且 RPM 達標才「送球」（FEED）
        */
        boolean yHeld = gamepad1.y;

        if (!yHeld) {
            // 沒按 Y：永遠吐球
            feederServo.setPower(FEEDER_OUTTAKE_POWER);
            feedEnabled = false;
        } else {
            // 按住 Y：需要 shooterOn + 達 RPM 才送球
            if (shooterOn) {
                if (!feedEnabled && rpm >= FEED_ON_RPM) {
                    feedEnabled = true;
                } else if (feedEnabled && rpm <= FEED_OFF_RPM) {
                    feedEnabled = false;
                }
                feederServo.setPower(feedEnabled ? FEEDER_FEED_POWER : 0.0);
            } else {
                feederServo.setPower(0.0);
                feedEnabled = false;
            }
        }

        /* ========= Intake（你指定的寫法） =========
           A：開
           B：關
        */
        if (gamepad1.a) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (gamepad1.b) {
            intakeMotor.setPower(0);
        }

        /* ========= Telemetry ========= */
        telemetry.addData("ShooterOn (X)", shooterOn);
        telemetry.addData("Y Held (Feed)", yHeld);
        telemetry.addData("Shooter RPM", "%.0f", rpm);
        telemetry.addData("FeedEnabled (RPM OK)", feedEnabled);
        telemetry.addData("Feeder Power", "%.2f",
                (!yHeld) ? FEEDER_OUTTAKE_POWER : (feedEnabled ? FEEDER_FEED_POWER : 0.0));
        telemetry.addData("Intake Power", intakeMotor.getPower());

        // 除錯：確認 RC 設定檔 motor type
        telemetry.addData("MotorType Ticks/Rev", shooterMotor.getMotorType().getTicksPerRev());
        telemetry.addData("MotorType MaxRPM", shooterMotor.getMotorType().getMaxRPM());

        telemetry.update();
    }
}