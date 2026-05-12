package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.ModuleConstants;

@TeleOp(name = "1a. Encoder Offset Reader", group = "Tuning")
public class _1a_EncoderOffsetReader extends LinearOpMode {

    private CRServo flServo, frServo, blServo, brServo;
    private AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;

    @Override
    public void runOpMode() {
        boolean initSuccess = true;
        StringBuilder errorMsg = new StringBuilder();

        // ===== 初始化 CRServo =====
        try { flServo = hardwareMap.get(CRServo.class, DriveConstants.kFrontLeftTurningMotorName);  flServo.setPower(0); }
        catch (Exception e) { initSuccess = false; errorMsg.append("FL Servo 找不到\n"); }

        try { frServo = hardwareMap.get(CRServo.class, DriveConstants.kFrontRightTurningMotorName); frServo.setPower(0); }
        catch (Exception e) { initSuccess = false; errorMsg.append("FR Servo 找不到\n"); }

        try { blServo = hardwareMap.get(CRServo.class, DriveConstants.kBackLeftTurningMotorName);   blServo.setPower(0); }
        catch (Exception e) { initSuccess = false; errorMsg.append("BL Servo 找不到\n"); }

        try { brServo = hardwareMap.get(CRServo.class, DriveConstants.kBackRightTurningMotorName);  brServo.setPower(0); }
        catch (Exception e) { initSuccess = false; errorMsg.append("BR Servo 找不到\n"); }

        // ===== 初始化絕對編碼器 =====
        try { flEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontLeftAbsoluteEncoderName); }
        catch (Exception e) { initSuccess = false; errorMsg.append("FL Encoder 找不到\n"); }

        try { frEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontRightAbsoluteEncoderName); }
        catch (Exception e) { initSuccess = false; errorMsg.append("FR Encoder 找不到\n"); }

        try { blEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackLeftAbsoluteEncoderName); }
        catch (Exception e) { initSuccess = false; errorMsg.append("BL Encoder 找不到\n"); }

        try { brEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackRightAbsoluteEncoderName); }
        catch (Exception e) { initSuccess = false; errorMsg.append("BR Encoder 找不到\n"); }

        // ===== Init 階段 =====
        while (!isStarted() && !isStopRequested()) {
            if (!initSuccess) {
                telemetry.addLine("⚠️ 硬體初始化失敗！");
                telemetry.addLine(errorMsg.toString());
            } else {
                telemetry.addLine("硬體正常，按 START 開始監測");
                telemetry.addLine("");
                showAllData();
            }
            telemetry.update();
            idle();
        }

        if (!initSuccess) return;

        // ===== 主迴圈 =====
        while (opModeIsActive()) {
            showAllData();
            telemetry.update();
        }
    }

    private void showAllData() {

        // ===== 從 Constants 讀取目前設定 =====
        double gearRatio    = ModuleConstants.kTurningMotorGearRatio;
        double flOffsetDeg  = DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg;
        double frOffsetDeg  = DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg;
        double blOffsetDeg  = DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg;
        double brOffsetDeg  = DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg;
        boolean flRev = DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed;
        boolean frRev = DriveConstants.kFrontRightDriveAbsoluteEncoderReversed;
        boolean blRev = DriveConstants.kBackLeftDriveAbsoluteEncoderReversed;
        boolean brRev = DriveConstants.kBackRightDriveAbsoluteEncoderReversed;

        // ===== 計算各層數值 =====
        double flRaw     = getRawDeg(flEncoder);
        double frRaw     = getRawDeg(frEncoder);
        double blRaw     = getRawDeg(blEncoder);
        double brRaw     = getRawDeg(brEncoder);

        double flGeared  = flRaw * gearRatio;
        double frGeared  = frRaw * gearRatio;
        double blGeared  = blRaw * gearRatio;
        double brGeared  = brRaw * gearRatio;

        double flFinal   = computeFinalDeg(flEncoder, flOffsetDeg, flRev);
        double frFinal   = computeFinalDeg(frEncoder, frOffsetDeg, frRev);
        double blFinal   = computeFinalDeg(blEncoder, blOffsetDeg, blRev);
        double brFinal   = computeFinalDeg(brEncoder, brOffsetDeg, brRev);

        // ===== 顯示目前 Constants 參數 =====
        telemetry.addLine("══ Constants 設定 ══");
        telemetry.addData("kTurningMotorGearRatio", "%.4f", gearRatio);
        telemetry.addData("Offset (FL/FR/BL/BR)",
                "%.1f° / %.1f° / %.1f° / %.1f°",
                flOffsetDeg, frOffsetDeg, blOffsetDeg, brOffsetDeg);
        telemetry.addData("Reversed (FL/FR/BL/BR)",
                "%b / %b / %b / %b", flRev, frRev, blRev, brRev);
        telemetry.addLine("");

        // ===== 第一層：原始角度（編碼器直接輸出）=====
        telemetry.addLine("── 第1層：Raw（編碼器直讀，不含任何處理）──");
        telemetry.addData("FL Raw", "%.2f°", flRaw);
        telemetry.addData("FR Raw", "%.2f°", frRaw);
        telemetry.addData("BL Raw", "%.2f°", blRaw);
        telemetry.addData("BR Raw", "%.2f°", brRaw);
        telemetry.addLine("");

        // ===== 第二層：乘以齒輪比之後（Offset 之前）=====
        telemetry.addLine("── 第2層：Raw × GearRatio（套用齒輪比，扣 Offset 前）──");
        telemetry.addData("FL Geared", "%.2f°", flGeared);
        telemetry.addData("FR Geared", "%.2f°", frGeared);
        telemetry.addData("BL Geared", "%.2f°", blGeared);
        telemetry.addData("BR Geared", "%.2f°", brGeared);
        telemetry.addLine("");

        // ===== 第三層：SwerveModule 實際輸入值（套用所有計算後）=====
        telemetry.addLine("── 第3層：SwerveModule 最終輸入（含 Offset、Reversed、Normalize）──");
        telemetry.addData("FL Final", "%.2f°  [%.3f rad]", flFinal, Math.toRadians(flFinal));
        telemetry.addData("FR Final", "%.2f°  [%.3f rad]", frFinal, Math.toRadians(frFinal));
        telemetry.addData("BL Final", "%.2f°  [%.3f rad]", blFinal, Math.toRadians(blFinal));
        telemetry.addData("BR Final", "%.2f°  [%.3f rad]", brFinal, Math.toRadians(brFinal));
        telemetry.addLine("");

        // ===== Voltage 原始值（方便 debug 斷線/接觸不良）=====
        telemetry.addLine("── Voltage ──");
        telemetry.addData("FL", "%.3fV  MaxV=%.2fV",
                flEncoder != null ? flEncoder.getVoltage() : -1,
                flEncoder != null ? flEncoder.getMaxVoltage() : -1);
        telemetry.addData("FR", "%.3fV", frEncoder != null ? frEncoder.getVoltage() : -1);
        telemetry.addData("BL", "%.3fV", blEncoder != null ? blEncoder.getVoltage() : -1);
        telemetry.addData("BR", "%.3fV", brEncoder != null ? brEncoder.getVoltage() : -1);
    }

    // ===== 第一層：原始角度（0~360°，不含任何處理）=====
    private double getRawDeg(AnalogInput enc) {
        if (enc == null) return 0;
        double maxV = enc.getMaxVoltage();
        if (maxV == 0) return 0;
        return (enc.getVoltage() / maxV) * 360.0;
    }

    // ===== 第三層：完整複製 SwerveModule.getAbsoluteEncoderRad() 邏輯 =====
    private double computeFinalDeg(AnalogInput enc, double offsetDeg, boolean reversed) {
        if (enc == null) return 0;
        double angle = enc.getVoltage() / enc.getMaxVoltage();
        angle *= 2.0 * Math.PI;                                      // raw rad
        angle *= ModuleConstants.kTurningMotorGearRatio;             // 齒輪比
        angle -= Math.toRadians(offsetDeg);                          // 扣 offset
        if (reversed) angle = -angle;
        while (angle >  Math.PI) angle -= 2.0 * Math.PI;            // normalize -π~π
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return Math.toDegrees(angle);                                // 轉回角度方便讀
    }
}