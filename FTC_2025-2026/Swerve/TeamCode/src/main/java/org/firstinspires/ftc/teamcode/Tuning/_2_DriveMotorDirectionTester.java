package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import android.content.Context;
import android.content.SharedPreferences;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/**
 * ════════════════════════════════════════
 *  2. Drive Motor Direction Tester
 * ════════════════════════════════════════
 *
 * 目的：確認四顆驅動馬達的方向設定是否正確
 *
 * 使用步驟：
 *   Step 1│ 確認 1c 已設好 Offset（輪子朝前時 Final = 0°）
 *   Step 2│ Init 階段等待輪子對齊到 0°（朝前）
 *   Step 3│ 按 Start，按 A 讓所有輪子前進
 *   Step 4│ 觀察 Encoder 數值：
 *            前進時數值應增加（↑）→ 方向正確 ✅
 *            前進時數值減少（↓）  → 需要 Reverse ⚠️
 *   Step 5│ 把需要 Reverse 的輪子改成 true：
 *            Constants.java → kXXXDriveEncoderReversed = true
 *
 * 按鍵對照：
 *   A        = 全部前進（主要測試）
 *   B        = 全部後退（驗證用）
 *   X        = 左側前進（FL + BL）
 *   Y        = 右側前進（FR + BR）
 *   DPAD ↑   = 只有 FL
 *   DPAD →   = 只有 FR
 *   DPAD ←   = 只有 BL
 *   DPAD ↓   = 只有 BR
 *   LB / RB  = 調整測試功率 ±10%
 */
@Config
@TeleOp(name = "2. Drive Motor Direction Tester", group = "Tuning")
public class _2_DriveMotorDirectionTester extends LinearOpMode {

    public static double driveTestPower = 0.3;

    // 硬體
    private DcMotorEx flDrive, frDrive, blDrive, brDrive;
    private CRServo   flTurn,  frTurn,  blTurn,  brTurn;
    private AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;

    // 轉向 PID
    private PIDController flPid, frPid, blPid, brPid;

    private FtcDashboard dashboard;

    // 按鍵防抖
    private boolean lastLb = false;
    private boolean lastRb = false;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        initHardware();

        // ══════════════════════════════
        //  Init 階段：對齊輪子到 0°
        // ══════════════════════════════
        while (!isStarted() && !isStopRequested()) {
            double flDeg = getWheelDeg(flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);
            double frDeg = getWheelDeg(frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);
            double blDeg = getWheelDeg(blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);
            double brDeg = getWheelDeg(brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);

            flTurn.setPower(turningPID(flPid, flDeg));
            frTurn.setPower(turningPID(frPid, frDeg));
            blTurn.setPower(turningPID(blPid, blDeg));
            brTurn.setPower(turningPID(brPid, brDeg));

            boolean flOk = Math.abs(flDeg) < 5;
            boolean frOk = Math.abs(frDeg) < 5;
            boolean blOk = Math.abs(blDeg) < 5;
            boolean brOk = Math.abs(brDeg) < 5;
            boolean allOk = flOk && frOk && blOk && brOk;

            telemetry.addLine("════ Drive Motor Direction Tester ════");
            telemetry.addLine("Init 階段：輪子對齊到朝前（0°）");
            telemetry.addLine("對齊完成後按 Start 開始測試");
            telemetry.addLine("");
            telemetry.addLine("── 輪子角度（目標：0°）──");
            telemetry.addData("FL", "%.1f°  %s", flDeg, flOk ? "✅" : "⏳");
            telemetry.addData("FR", "%.1f°  %s", frDeg, frOk ? "✅" : "⏳");
            telemetry.addData("BL", "%.1f°  %s", blDeg, blOk ? "✅" : "⏳");
            telemetry.addData("BR", "%.1f°  %s", brDeg, brOk ? "✅" : "⏳");
            telemetry.addLine("");
            telemetry.addLine(allOk ? "✅ 全部對齊，可以按 Start！" : "⏳ 等待對齊中...");

            sendDashboardAngles(flDeg, frDeg, blDeg, brDeg);
            telemetry.update();
            idle();
        }

        resetDriveEncoders();
        if (!opModeIsActive()) return;

        // ══════════════════════════════
        //  Start 階段：測試驅動馬達
        // ══════════════════════════════
        while (opModeIsActive()) {

            // 調整功率
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            if (rb && !lastRb) driveTestPower = Math.min(1.0, driveTestPower + 0.1);
            if (lb && !lastLb) driveTestPower = Math.max(0.1, driveTestPower - 0.1);
            lastRb = rb; lastLb = lb;

            // 持續維持轉向對齊
            double flDeg = getWheelDeg(flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);
            double frDeg = getWheelDeg(frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);
            double blDeg = getWheelDeg(blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);
            double brDeg = getWheelDeg(brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);
            flTurn.setPower(turningPID(flPid, flDeg));
            frTurn.setPower(turningPID(frPid, frDeg));
            blTurn.setPower(turningPID(blPid, blDeg));
            brTurn.setPower(turningPID(brPid, brDeg));

            // 驅動馬達控制
            String action = "無（放開所有按鈕）";
            double fl = 0, fr = 0, bl = 0, br = 0;

            if      (gamepad1.a)          { fl=fr=bl=br= driveTestPower; action = "A → 全部前進"; }
            else if (gamepad1.b)          { fl=fr=bl=br=-driveTestPower; action = "B → 全部後退"; }
            else if (gamepad1.x)          { fl=bl= driveTestPower;       action = "X → 左側前進 (FL+BL)"; }
            else if (gamepad1.y)          { fr=br= driveTestPower;       action = "Y → 右側前進 (FR+BR)"; }
            else if (gamepad1.dpad_up)    { fl=    driveTestPower;       action = "↑ → FL 單獨前進"; }
            else if (gamepad1.dpad_right) { fr=    driveTestPower;       action = "→ → FR 單獨前進"; }
            else if (gamepad1.dpad_left)  { bl=    driveTestPower;       action = "← → BL 單獨前進"; }
            else if (gamepad1.dpad_down)  { br=    driveTestPower;       action = "↓ → BR 單獨前進"; }

            flDrive.setPower(fl);
            frDrive.setPower(fr);
            blDrive.setPower(bl);
            brDrive.setPower(br);

            // Encoder 數值
            int flPos = flDrive.getCurrentPosition();
            int frPos = frDrive.getCurrentPosition();
            int blPos = blDrive.getCurrentPosition();
            int brPos = brDrive.getCurrentPosition();

            // 判斷方向結論
            String flResult = dirResult(fl, flPos);
            String frResult = dirResult(fr, frPos);
            String blResult = dirResult(bl, blPos);
            String brResult = dirResult(br, brPos);

            boolean anyReverse = flResult.contains("⚠️") || frResult.contains("⚠️")
                    || blResult.contains("⚠️") || brResult.contains("⚠️");

            // Telemetry
            telemetry.addLine("════ Drive Motor Direction Tester ════");
            telemetry.addData("目前動作", action);
            telemetry.addData("測試功率", "%.0f%%  (LB/RB 調整)", driveTestPower * 100);
            telemetry.addLine("");

            telemetry.addLine("── Encoder 數值（前進應增加↑）──");
            telemetry.addData("FL", "%6d  %s", flPos, flResult);
            telemetry.addData("FR", "%6d  %s", frPos, frResult);
            telemetry.addData("BL", "%6d  %s", blPos, blResult);
            telemetry.addData("BR", "%6d  %s", brPos, brResult);
            telemetry.addLine("");

            telemetry.addLine("── 輪子角度（應維持 0°）──");
            telemetry.addData("FL/FR/BL/BR", "%.1f° / %.1f° / %.1f° / %.1f°",
                    flDeg, frDeg, blDeg, brDeg);
            telemetry.addLine("");

            if (anyReverse) {
                telemetry.addLine("⚠️ 需要 Reverse 的輪子請改 Constants.java：");
                telemetry.addLine("   kXXXDriveEncoderReversed = true");
            } else if (!action.equals("無（放開所有按鈕）")) {
                telemetry.addLine("✅ 全部方向正確！");
            } else {
                telemetry.addLine("── 按鍵說明 ──");
                telemetry.addLine("A=全部前進  B=全部後退");
                telemetry.addLine("X=左側  Y=右側");
                telemetry.addLine("↑=FL  →=FR  ←=BL  ↓=BR");
            }

            sendDashboardEncoders(flPos, frPos, blPos, brPos);
            telemetry.update();
        }

        stopAll();
    }

    /**
     * 判斷驅動方向是否正確
     * 有給功率且 |pos| > 30 才判斷，避免靜止時誤判
     */
    private String dirResult(double power, int pos) {
        if (Math.abs(power) < 0.01) return "──";
        if (Math.abs(pos) < 30)     return "測量中...";
        boolean correct = (power > 0 && pos > 0) || (power < 0 && pos < 0);
        return correct ? "✅ 正確" : "⚠️ 需要 Reverse";
    }

    private void initHardware() {
        flDrive = hardwareMap.get(DcMotorEx.class, DriveConstants.kFrontLeftDriveMotorName);
        frDrive = hardwareMap.get(DcMotorEx.class, DriveConstants.kFrontRightDriveMotorName);
        blDrive = hardwareMap.get(DcMotorEx.class, DriveConstants.kBackLeftDriveMotorName);
        brDrive = hardwareMap.get(DcMotorEx.class, DriveConstants.kBackRightDriveMotorName);

        flDrive.setDirection(DriveConstants.kFrontLeftDriveEncoderReversed  ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frDrive.setDirection(DriveConstants.kFrontRightDriveEncoderReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        blDrive.setDirection(DriveConstants.kBackLeftDriveEncoderReversed   ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        brDrive.setDirection(DriveConstants.kBackRightDriveEncoderReversed  ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();

        flTurn = hardwareMap.get(CRServo.class, DriveConstants.kFrontLeftTurningMotorName);
        frTurn = hardwareMap.get(CRServo.class, DriveConstants.kFrontRightTurningMotorName);
        blTurn = hardwareMap.get(CRServo.class, DriveConstants.kBackLeftTurningMotorName);
        brTurn = hardwareMap.get(CRServo.class, DriveConstants.kBackRightTurningMotorName);

        flTurn.setDirection(DriveConstants.kFrontLeftTurningEncoderReversed  ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        frTurn.setDirection(DriveConstants.kFrontRightTurningEncoderReversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        blTurn.setDirection(DriveConstants.kBackLeftTurningEncoderReversed   ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        brTurn.setDirection(DriveConstants.kBackRightTurningEncoderReversed  ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        flEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontLeftAbsoluteEncoderName);
        frEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontRightAbsoluteEncoderName);
        blEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackLeftAbsoluteEncoderName);
        brEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackRightAbsoluteEncoderName);

        double p = Constants.ModuleConstants.kPTurning;
        double i = Constants.ModuleConstants.kITurning;
        double d = Constants.ModuleConstants.kDTurning;
        flPid = new PIDController(p, i, d);
        frPid = new PIDController(p, i, d);
        blPid = new PIDController(p, i, d);
        brPid = new PIDController(p, i, d);
    }

    private void resetDriveEncoders() {
        for (DcMotorEx m : new DcMotorEx[]{flDrive, frDrive, blDrive, brDrive}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** 讀取車輪角度（扣 offset，normalize 到 -90~90°）*/
    private double getWheelDeg(AnalogInput enc, double offsetDeg) {
        double angle = (enc.getVoltage() / enc.getMaxVoltage()) * 360.0 - offsetDeg;
        while (angle >  90) angle -= 180;
        while (angle < -90) angle += 180;
        return angle;
    }

    /** 轉向 PID 輸出 */
    private double turningPID(PIDController pid, double currentDeg) {
        double error = -currentDeg;
        while (error >  90) error -= 180;
        while (error < -90) error += 180;
        double out = pid.calculate(0, Math.toRadians(error));
        out = Math.max(-0.8, Math.min(0.8, out));
        if (Math.abs(error) < 3) out *= 0.5;
        return out;
    }

    private void sendDashboardAngles(double fl, double fr, double bl, double br) {
        TelemetryPacket p = new TelemetryPacket();
        p.put("FL_deg", fl); p.put("FR_deg", fr);
        p.put("BL_deg", bl); p.put("BR_deg", br);
        p.put("target", 0);
        dashboard.sendTelemetryPacket(p);
    }

    private void sendDashboardEncoders(int fl, int fr, int bl, int br) {
        TelemetryPacket p = new TelemetryPacket();
        p.put("FL_enc", fl); p.put("FR_enc", fr);
        p.put("BL_enc", bl); p.put("BR_enc", br);
        dashboard.sendTelemetryPacket(p);
    }

    private void stopAll() {
        flDrive.setPower(0); frDrive.setPower(0);
        blDrive.setPower(0); brDrive.setPower(0);
        flTurn.setPower(0);  frTurn.setPower(0);
        blTurn.setPower(0);  brTurn.setPower(0);

        // ★ 清除累積角度，讓 Swerve_Control 下次啟動用絕對編碼器重新計算
        SharedPreferences anglePrefs = AppUtil.getInstance().getRootActivity()
                .getSharedPreferences("SwerveModulePrefs", Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = anglePrefs.edit();
        for (String name : new String[]{
                DriveConstants.kFrontLeftTurningMotorName,
                DriveConstants.kFrontRightTurningMotorName,
                DriveConstants.kBackLeftTurningMotorName,
                DriveConstants.kBackRightTurningMotorName}) {
            editor.remove("swerve_angle_" + name);
            editor.remove("swerve_angle_" + name + "_raw");
        }
        editor.apply();
    }
}