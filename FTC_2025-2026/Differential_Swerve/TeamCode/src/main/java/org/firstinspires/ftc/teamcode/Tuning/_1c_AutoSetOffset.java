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
 * 2. 按 A 將四輪 offset 寫入 Control Hub
 * 3. 同時把四輪累積角追蹤歸零（當下視為輪角 0°）
 * 4. 重新啟動 Swerve_Control 驗證 Final 是否接近 0°
 *
 * 注意：此程式會覆蓋 Control Hub 上既有 offset/tracking 記錄。
 */
@TeleOp(name = "1c. Auto Set Offset", group = "Tuning")
public class _1c_AutoSetOffset extends LinearOpMode {

    private CRServo flServo, frServo, blServo, brServo;
    private AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;
    private SharedPreferences offsetPrefs;
    private SharedPreferences trackingPrefs;

    private boolean lastA = false;

    @Override
    public void runOpMode() {
        offsetPrefs = AppUtil.getDefContext().getSharedPreferences("SwerveOffsetPrefs", Context.MODE_PRIVATE);
        trackingPrefs = AppUtil.getDefContext().getSharedPreferences("SwerveModulePersistentAngles", Context.MODE_PRIVATE);

        // ===== 初始化硬體 =====
        boolean ok = true;
        StringBuilder err = new StringBuilder();

        try { flServo = hardwareMap.get(CRServo.class, DriveConstants.kFrontLeftTurningMotorName);  flServo.setPower(0); }
        catch (Exception e) { ok = false; err.append("FL Servo 找不到\n"); }
        try { frServo = hardwareMap.get(CRServo.class, DriveConstants.kFrontRightTurningMotorName); frServo.setPower(0); }
        catch (Exception e) { ok = false; err.append("FR Servo 找不到\n"); }
        try { blServo = hardwareMap.get(CRServo.class, DriveConstants.kBackLeftTurningMotorName);   blServo.setPower(0); }
        catch (Exception e) { ok = false; err.append("BL Servo 找不到\n"); }
        try { brServo = hardwareMap.get(CRServo.class, DriveConstants.kBackRightTurningMotorName);  brServo.setPower(0); }
        catch (Exception e) { ok = false; err.append("BR Servo 找不到\n"); }

        try { flEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontLeftAbsoluteEncoderName); }
        catch (Exception e) { ok = false; err.append("FL Encoder 找不到\n"); }
        try { frEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontRightAbsoluteEncoderName); }
        catch (Exception e) { ok = false; err.append("FR Encoder 找不到\n"); }
        try { blEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackLeftAbsoluteEncoderName); }
        catch (Exception e) { ok = false; err.append("BL Encoder 找不到\n"); }
        try { brEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackRightAbsoluteEncoderName); }
        catch (Exception e) { ok = false; err.append("BR Encoder 找不到\n"); }

        if (!ok) {
            telemetry.addLine("⚠️ 硬體初始化失敗");
            telemetry.addLine(err.toString());
            telemetry.update();
            waitForStart();
            return;
        }

        waitForStart();

        // ===== 主迴圈 =====
        while (opModeIsActive()) {

            double flGeared = getGearedDeg(flEncoder);
            double frGeared = getGearedDeg(frEncoder);
            double blGeared = getGearedDeg(blEncoder);
            double brGeared = getGearedDeg(brEncoder);

            // 套用目前 offset 後的 Final 角度（確認用）
            double flFinal = getFinalDeg(flEncoder);
            double frFinal = getFinalDeg(frEncoder);
            double blFinal = getFinalDeg(blEncoder);
            double brFinal = getFinalDeg(brEncoder);

            boolean allNearZero = Math.abs(flFinal) < 5 && Math.abs(frFinal) < 5
                    && Math.abs(blFinal) < 5 && Math.abs(brFinal) < 5;

            // ===== Telemetry =====
            telemetry.addLine("════ 步驟說明 ════");
            telemetry.addLine("1. 手動把四個輪子轉到朝向正前方");
            telemetry.addLine("2. 按 A 寫入 Control Hub offset");
            telemetry.addLine("3. 同步重設累積角=0（當下即輪角0°）");
            telemetry.addLine("");

            telemetry.addLine("── 第2層 Geared（這就是要填入的 Offset）──");
            telemetry.addData("FL", "%.2f°", flGeared);
            telemetry.addData("FR", "%.2f°", frGeared);
            telemetry.addData("BL", "%.2f°", blGeared);
            telemetry.addData("BR", "%.2f°", brGeared);
            telemetry.addLine("");

            telemetry.addLine("── 第3層 Final（套用 Offset 後，應接近 0°）──");
            telemetry.addData("FL", "%.2f°  %s", flFinal, Math.abs(flFinal) < 5 ? "✅" : "❌");
            telemetry.addData("FR", "%.2f°  %s", frFinal, Math.abs(frFinal) < 5 ? "✅" : "❌");
            telemetry.addData("BL", "%.2f°  %s", blFinal, Math.abs(blFinal) < 5 ? "✅" : "❌");
            telemetry.addData("BR", "%.2f°  %s", brFinal, Math.abs(brFinal) < 5 ? "✅" : "❌");
            telemetry.addLine("");

            telemetry.addLine("── Control Hub 目前 offset（度）──");
            telemetry.addData("FL offset", "%.2f°", Math.toDegrees(loadOffsetRad(DriveConstants.kFrontLeftTurningMotorName, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad)));
            telemetry.addData("FR offset", "%.2f°", Math.toDegrees(loadOffsetRad(DriveConstants.kFrontRightTurningMotorName, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad)));
            telemetry.addData("BL offset", "%.2f°", Math.toDegrees(loadOffsetRad(DriveConstants.kBackLeftTurningMotorName, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad)));
            telemetry.addData("BR offset", "%.2f°", Math.toDegrees(loadOffsetRad(DriveConstants.kBackRightTurningMotorName, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad)));
            telemetry.addLine("");

            if (allNearZero) {
                telemetry.addLine("✅ 全部對齊，Constants 可維持不變");
            } else {
                telemetry.addLine("⏳ 繼續調整輪子方向...");
            }

            // ===== A 鍵：寫入 offset + 重置累積角 =====
            boolean a = gamepad1.a;
            if (a && !lastA) {
                saveOffsetAndZeroTracking(DriveConstants.kFrontLeftTurningMotorName, flEncoder);
                saveOffsetAndZeroTracking(DriveConstants.kFrontRightTurningMotorName, frEncoder);
                saveOffsetAndZeroTracking(DriveConstants.kBackLeftTurningMotorName, blEncoder);
                saveOffsetAndZeroTracking(DriveConstants.kBackRightTurningMotorName, brEncoder);

                telemetry.addLine("✅ 已寫入 Control Hub offset");
                telemetry.addLine("✅ 已重設四輪累積角追蹤為 0");
                telemetry.addLine("請重新啟動 Swerve_Control 生效");
                telemetry.update();
                sleep(1500);
            }
            lastA = a;

            telemetry.update();
        }
    }

    // ===== 計算 =====
    private double getGearedDeg(AnalogInput enc) {
        if (enc == null) return 0;
        return (enc.getVoltage() / enc.getMaxVoltage()) * 360.0 * ModuleConstants.kTurningMotorGearRatio;
    }

    private double getFinalDeg(AnalogInput enc) {
        if (enc == null) return 0;
        double angle = Math.toRadians(getGearedDeg(enc));
        angle -= getActiveOffsetRadForEncoder(enc);
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return Math.toDegrees(angle);
    }

    private double getActiveOffsetRadForEncoder(AnalogInput enc) {
        if (enc == flEncoder) {
            return loadOffsetRad(DriveConstants.kFrontLeftTurningMotorName, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad);
        }
        if (enc == frEncoder) {
            return loadOffsetRad(DriveConstants.kFrontRightTurningMotorName, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad);
        }
        if (enc == blEncoder) {
            return loadOffsetRad(DriveConstants.kBackLeftTurningMotorName, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);
        }
        return loadOffsetRad(DriveConstants.kBackRightTurningMotorName, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad);
    }

    private void saveOffsetAndZeroTracking(String servoName, AnalogInput encoder) {
        double rawServoRad = getRawServoRad(encoder);
        double wheelOffsetRad = rawServoRad * ModuleConstants.kTurningMotorGearRatio;

        offsetPrefs.edit()
                .putFloat("offset_" + servoName, (float) wheelOffsetRad)
                .apply();

        trackingPrefs.edit()
                .putFloat("accum_angle_" + servoName, 0f)
                .putFloat("last_raw_" + servoName, (float) rawServoRad)
                .apply();
    }

    private double loadOffsetRad(String servoName, double fallback) {
        return offsetPrefs.getFloat("offset_" + servoName, (float) fallback);
    }

    private double getRawServoRad(AnalogInput enc) {
        if (enc == null) return 0;
        return (enc.getVoltage() / enc.getMaxVoltage()) * 2.0 * Math.PI;
    }
}