package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
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
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

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
    private FtcDashboard dashboard;

    private SwerveSubsystem swerve;

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
            SwerveModuleState forward = new SwerveModuleState(0.001, new Rotation2d(0));
            swerve.getFrontLeft().alignTurningOnly(0);
            swerve.getFrontRight().alignTurningOnly(0);
            swerve.getBackLeft().alignTurningOnly(0);
            swerve.getBackRight().alignTurningOnly(0);

            double flDeg = Math.toDegrees(swerve.getFrontLeft().getTurningPosition());
            double frDeg = Math.toDegrees(swerve.getFrontRight().getTurningPosition());
            double blDeg = Math.toDegrees(swerve.getBackLeft().getTurningPosition());
            double brDeg = Math.toDegrees(swerve.getBackRight().getTurningPosition());

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
            telemetry.addData("FL/FR/BL/BR", "%.1f° / %.1f° / %.1f° / %.1f°",
                    Math.toDegrees(swerve.getFrontLeft().getTurningPosition()),
                    Math.toDegrees(swerve.getFrontRight().getTurningPosition()),
                    Math.toDegrees(swerve.getBackLeft().getTurningPosition()),
                    Math.toDegrees(swerve.getBackRight().getTurningPosition()));
            telemetry.addLine("");
            telemetry.addLine(allOk ? "✅ 全部對齊，可以按 Start！" : "⏳ 等待對齊中...");

            sendDashboardAngles(flDeg, frDeg, blDeg, brDeg);
            telemetry.update();
            idle();
        }

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
            SwerveModuleState forward = new SwerveModuleState(0.001, new Rotation2d(0));
            swerve.getFrontLeft().alignTurningOnly(0);
            swerve.getFrontRight().alignTurningOnly(0);
            swerve.getBackLeft().alignTurningOnly(0);
            swerve.getBackRight().alignTurningOnly(0);

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

            swerve.getFrontLeft().setDriveMotorPowerDirect(fl);
            swerve.getFrontRight().setDriveMotorPowerDirect(fr);
            swerve.getBackLeft().setDriveMotorPowerDirect(bl);
            swerve.getBackRight().setDriveMotorPowerDirect(br);

            // Telemetry
            telemetry.addLine("════ Drive Motor Direction Tester ════");
            telemetry.addData("目前動作", action);
            telemetry.addData("測試功率", "%.0f%%  (LB/RB 調整)", driveTestPower * 100);
            telemetry.addLine("");

            telemetry.addLine("── Encoder 數值（前進應增加↑）──");
            telemetry.addLine("");

            telemetry.addLine("── 輪子角度（應維持 0°）──");
            telemetry.addData("FL/FR/BL/BR", "%.1f° / %.1f° / %.1f° / %.1f°",
                    Math.toDegrees(swerve.getFrontLeft().getTurningPosition()),
                    Math.toDegrees(swerve.getFrontRight().getTurningPosition()),
                    Math.toDegrees(swerve.getBackLeft().getTurningPosition()),
                    Math.toDegrees(swerve.getBackRight().getTurningPosition()));
            telemetry.addLine("");

            telemetry.addLine("── 按鍵說明 ──");
            telemetry.addLine("A=全部前進  B=全部後退");
            telemetry.addLine("X=左側  Y=右側");
            telemetry.addLine("↑=FL  →=FR  ←=BL  ↓=BR");
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
        swerve = new SwerveSubsystem(hardwareMap);
        swerve.getFrontLeft().disableSaving();
        swerve.getFrontRight().disableSaving();
        swerve.getBackLeft().disableSaving();
        swerve.getBackRight().disableSaving();
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
        // ★ 重新開啟儲存，讓 stop() 把目前 0° 的角度存進去
        swerve.getFrontLeft().enableSaving();
        swerve.getFrontRight().enableSaving();
        swerve.getBackLeft().enableSaving();
        swerve.getBackRight().enableSaving();

        swerve.stopModules();  // 內部呼叫 saveAccumulatedAngle()
    }
}