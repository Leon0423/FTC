package org.firstinspires.ftc.teamcode.Tuning;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.ModuleConstants;

/**
 * Offset 校正輔助（寫入 Control Hub）
 *
 * 使用步驟：
 * 1. 手動把四個輪子轉到朝向正前方
 * 2. 按 A 將四輪 offset 寫入 Control Hub 並重設累積角
 * 3. 按 B 清除所有已儲存的 offset 資料
 * 4. 重新啟動 Swerve_Control 驗證 Final 是否接近 0°
 */
@TeleOp(name = "1c. Auto Set Offset", group = "Tuning")
public class _1c_AutoSetOffset extends LinearOpMode {

    private static final String OFFSET_PREFS_NAME   = "SwerveOffsetPrefs";
    private static final String TRACKING_PREFS_NAME  = "SwerveModulePersistentAngles";
    private static final String LEGACY_PREFS_NAME    = "SwerveModuleState";

    private CRServo flServo, frServo, blServo, brServo;
    private AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;
    private SharedPreferences offsetPrefs;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {
        offsetPrefs = AppUtil.getDefContext()
                .getSharedPreferences(OFFSET_PREFS_NAME, Context.MODE_PRIVATE);

        boolean ok = true;
        StringBuilder err = new StringBuilder();

        try { flServo = hardwareMap.get(CRServo.class, DriveConstants.kFrontLeftTurningMotorName);  flServo.setPower(0); }
        catch (Exception e) { ok = false; err.append("FL Servo not found\n"); }
        try { frServo = hardwareMap.get(CRServo.class, DriveConstants.kFrontRightTurningMotorName); frServo.setPower(0); }
        catch (Exception e) { ok = false; err.append("FR Servo not found\n"); }
        try { blServo = hardwareMap.get(CRServo.class, DriveConstants.kBackLeftTurningMotorName);   blServo.setPower(0); }
        catch (Exception e) { ok = false; err.append("BL Servo not found\n"); }
        try { brServo = hardwareMap.get(CRServo.class, DriveConstants.kBackRightTurningMotorName);  brServo.setPower(0); }
        catch (Exception e) { ok = false; err.append("BR Servo not found\n"); }

        try { flEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontLeftAbsoluteEncoderName); }
        catch (Exception e) { ok = false; err.append("FL Encoder not found\n"); }
        try { frEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontRightAbsoluteEncoderName); }
        catch (Exception e) { ok = false; err.append("FR Encoder not found\n"); }
        try { blEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackLeftAbsoluteEncoderName); }
        catch (Exception e) { ok = false; err.append("BL Encoder not found\n"); }
        try { brEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackRightAbsoluteEncoderName); }
        catch (Exception e) { ok = false; err.append("BR Encoder not found\n"); }

        if (!ok) {
            telemetry.addLine("Hardware init failed:");
            telemetry.addLine(err.toString());
            telemetry.update();
            waitForStart();
            return;
        }

        waitForStart();

        while (opModeIsActive()) {

            double flGeared = getGearedDeg(flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
            double frGeared = getGearedDeg(frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
            double blGeared = getGearedDeg(blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
            double brGeared = getGearedDeg(brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

            double flFinal = getFinalDeg(flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
                    DriveConstants.kFrontLeftTurningMotorName, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad);
            double frFinal = getFinalDeg(frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
                    DriveConstants.kFrontRightTurningMotorName, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad);
            double blFinal = getFinalDeg(blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
                    DriveConstants.kBackLeftTurningMotorName, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);
            double brFinal = getFinalDeg(brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
                    DriveConstants.kBackRightTurningMotorName, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad);

            boolean allNearZero = Math.abs(flFinal) < 5 && Math.abs(frFinal) < 5
                    && Math.abs(blFinal) < 5 && Math.abs(brFinal) < 5;

            telemetry.addLine("=== Steps ===");
            telemetry.addLine("1. Manually point all wheels forward");
            telemetry.addLine("2. Press A to save offsets");
            telemetry.addLine("3. Press B to CLEAR all offsets");
            telemetry.addLine("");

            telemetry.addLine("-- Geared (this is the offset value) --");
            telemetry.addData("FL", "%.2f deg", flGeared);
            telemetry.addData("FR", "%.2f deg", frGeared);
            telemetry.addData("BL", "%.2f deg", blGeared);
            telemetry.addData("BR", "%.2f deg", brGeared);
            telemetry.addLine("");

            telemetry.addLine("-- Final (after offset, should be ~0) --");
            telemetry.addData("FL", "%.2f deg  %s", flFinal, check(flFinal));
            telemetry.addData("FR", "%.2f deg  %s", frFinal, check(frFinal));
            telemetry.addData("BL", "%.2f deg  %s", blFinal, check(blFinal));
            telemetry.addData("BR", "%.2f deg  %s", brFinal, check(brFinal));
            telemetry.addLine("");

            telemetry.addLine("-- Stored offsets (deg) --");
            telemetry.addData("FL", "%.2f", Math.toDegrees(loadOffset(DriveConstants.kFrontLeftTurningMotorName,  DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad)));
            telemetry.addData("FR", "%.2f", Math.toDegrees(loadOffset(DriveConstants.kFrontRightTurningMotorName, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad)));
            telemetry.addData("BL", "%.2f", Math.toDegrees(loadOffset(DriveConstants.kBackLeftTurningMotorName,   DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad)));
            telemetry.addData("BR", "%.2f", Math.toDegrees(loadOffset(DriveConstants.kBackRightTurningMotorName,  DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad)));
            telemetry.addLine("");

            if (allNearZero) {
                telemetry.addLine(">> All aligned! Offset looks good.");
            } else {
                telemetry.addLine(">> Keep adjusting wheel direction...");
            }

            // A: save offsets
            boolean a = gamepad1.a;
            if (a && !lastA) {
                saveOffset(DriveConstants.kFrontLeftTurningMotorName,  flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
                saveOffset(DriveConstants.kFrontRightTurningMotorName, frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
                saveOffset(DriveConstants.kBackLeftTurningMotorName,   blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
                saveOffset(DriveConstants.kBackRightTurningMotorName,  brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

                clearTrackingPrefs();

                telemetry.addLine(">> SAVED offsets to Control Hub");
                telemetry.addLine(">> Cleared tracking data");
                telemetry.addLine(">> Restart Swerve_Control to apply");
                telemetry.update();
                sleep(1500);
            }
            lastA = a;

            // B: clear all offsets
            boolean b = gamepad1.b;
            if (b && !lastB) {
                offsetPrefs.edit().clear().apply();

                clearTrackingPrefs();

                telemetry.addLine(">> CLEARED all stored offsets");
                telemetry.addLine(">> CLEARED all tracking data");
                telemetry.addLine(">> Will fall back to Constants values");
                telemetry.update();
                sleep(1500);
            }
            lastB = b;

            telemetry.update();
        }
    }

    // ── Helpers ──────────────────────────────────────────────────────────

    private double getRawServoRad(AnalogInput enc) {
        return (enc.getVoltage() / ModuleConstants.kTurningEncoderMaxVoltage) * 2.0 * Math.PI;
    }

    private double getGearedDeg(AnalogInput enc, boolean reversed) {
        double sign = reversed ? -1.0 : 1.0;
        return sign * (enc.getVoltage() / ModuleConstants.kTurningEncoderMaxVoltage)
                * 360.0 * ModuleConstants.kTurningMotorGearRatio;
    }

    private double getFinalDeg(AnalogInput enc, boolean reversed,
                               String servoName, double fallbackOffsetRad) {
        double rawRad = getRawServoRad(enc);
        double angle = (reversed ? -rawRad : rawRad) * ModuleConstants.kTurningMotorGearRatio;
        angle -= loadOffset(servoName, fallbackOffsetRad);
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return Math.toDegrees(angle);
    }

    private void saveOffset(String servoName, AnalogInput enc, boolean reversed) {
        double rawRad = getRawServoRad(enc);
        double orientedRad = reversed ? -rawRad : rawRad;
        double wheelOffsetRad = orientedRad * ModuleConstants.kTurningMotorGearRatio;
        offsetPrefs.edit()
                .putFloat("offset_" + servoName, (float) wheelOffsetRad)
                .apply();
    }

    private double loadOffset(String servoName, double fallback) {
        return offsetPrefs.getFloat("offset_" + servoName, (float) fallback);
    }

    private void clearTrackingPrefs() {
        AppUtil.getDefContext()
                .getSharedPreferences(TRACKING_PREFS_NAME, Context.MODE_PRIVATE)
                .edit().clear().apply();
        hardwareMap.appContext
                .getSharedPreferences(LEGACY_PREFS_NAME, Context.MODE_PRIVATE)
                .edit().clear().apply();
    }

    private static String check(double deg) {
        return Math.abs(deg) < 5 ? "OK" : "X";
    }
}
