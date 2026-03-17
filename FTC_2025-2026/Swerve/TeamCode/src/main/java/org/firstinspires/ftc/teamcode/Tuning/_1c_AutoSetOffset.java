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
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * 自動設定 Offset 並寫入 SharedPreferences
 *
 * 使用步驟：
 * 1. 手動把四個輪子轉到朝向正前方
 * 2. 確認第3層 Final 接近 0° 後按 A
 * 3. Offset 自動寫入 SharedPreferences
 * 4. 重新啟動 Swerve_Control 即可
 *
 * 注意：此程式會覆蓋 SharedPreferences 的 offset，
 *       但不會修改 Constants.java，兩者獨立。
 */
@TeleOp(name = "1c. Auto Set Offset", group = "Tuning")
public class _1c_AutoSetOffset extends LinearOpMode {

    private CRServo flServo, frServo, blServo, brServo;
    private AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;

    private SharedPreferences offsetPrefs;
    private SharedPreferences anglePrefs;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {
        offsetPrefs = AppUtil.getInstance().getRootActivity()
                .getSharedPreferences("SwerveOffsetPrefs", Context.MODE_PRIVATE);
        anglePrefs = AppUtil.getInstance().getRootActivity()
                .getSharedPreferences("SwerveModulePrefs", Context.MODE_PRIVATE);

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
            telemetry.addLine("2. 確認 Final 接近 0° 後按 A 寫入");
            telemetry.addLine("3. 按 B 可清除所有 SharedPreferences");
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

            // SharedPreferences 現有值
            telemetry.addLine("── SharedPreferences 目前儲存值 ──");
            telemetry.addData("FL offset", "%.2f°", Math.toDegrees(offsetPrefs.getFloat(
                    "offset_" + DriveConstants.kFrontLeftTurningMotorName, Float.MAX_VALUE)));
            telemetry.addData("FR offset", "%.2f°", Math.toDegrees(offsetPrefs.getFloat(
                    "offset_" + DriveConstants.kFrontRightTurningMotorName, Float.MAX_VALUE)));
            telemetry.addData("BL offset", "%.2f°", Math.toDegrees(offsetPrefs.getFloat(
                    "offset_" + DriveConstants.kBackLeftTurningMotorName, Float.MAX_VALUE)));
            telemetry.addData("BR offset", "%.2f°", Math.toDegrees(offsetPrefs.getFloat(
                    "offset_" + DriveConstants.kBackRightTurningMotorName, Float.MAX_VALUE)));
            telemetry.addLine("");

            if (allNearZero) {
                telemetry.addLine("✅ 全部對齊，可以按 A 寫入");
            } else {
                telemetry.addLine("⏳ 繼續調整輪子方向...");
            }

            // ===== A 鍵：寫入 Offset =====
            boolean a = gamepad1.a;
            if (a && !lastA) {
                saveOffset(DriveConstants.kFrontLeftTurningMotorName,  getGearedRad(flEncoder));
                saveOffset(DriveConstants.kFrontRightTurningMotorName, getGearedRad(frEncoder));
                saveOffset(DriveConstants.kBackLeftTurningMotorName,   getGearedRad(blEncoder));
                saveOffset(DriveConstants.kBackRightTurningMotorName,  getGearedRad(brEncoder));

                // 清除角度記憶，讓下次啟動以 0° 為基準重新開始
                clearAngle(DriveConstants.kFrontLeftTurningMotorName);
                clearAngle(DriveConstants.kFrontRightTurningMotorName);
                clearAngle(DriveConstants.kBackLeftTurningMotorName);
                clearAngle(DriveConstants.kBackRightTurningMotorName);

                telemetry.addLine("✅ Offset 已寫入 SharedPreferences！");
                telemetry.addLine("請重新啟動 Swerve_Control");
                telemetry.update();
                sleep(2000);
                return;
            }
            lastA = a;

            // ===== B 鍵：清除所有 SharedPreferences =====
            boolean b = gamepad1.b;
            if (b && !lastB) {
                clearOffset(DriveConstants.kFrontLeftTurningMotorName);
                clearOffset(DriveConstants.kFrontRightTurningMotorName);
                clearOffset(DriveConstants.kBackLeftTurningMotorName);
                clearOffset(DriveConstants.kBackRightTurningMotorName);
                clearAngle(DriveConstants.kFrontLeftTurningMotorName);
                clearAngle(DriveConstants.kFrontRightTurningMotorName);
                clearAngle(DriveConstants.kBackLeftTurningMotorName);
                clearAngle(DriveConstants.kBackRightTurningMotorName);
                telemetry.addLine("🗑 所有 SharedPreferences 已清除");
                telemetry.update();
                sleep(1000);
            }
            lastB = b;

            telemetry.update();
        }
    }

    // ===== 計算 =====
    private double getGearedDeg(AnalogInput enc) {
        if (enc == null) return 0;
        return (enc.getVoltage() / enc.getMaxVoltage()) * 360.0 * ModuleConstants.kTurningMotorGearRatio;
    }

    private double getGearedRad(AnalogInput enc) {
        if (enc == null) return 0;
        return (enc.getVoltage() / enc.getMaxVoltage()) * 2 * Math.PI * ModuleConstants.kTurningMotorGearRatio;
    }

    // 套用目前 SharedPreferences 的 offset 計算 Final
    private double getFinalDeg(AnalogInput enc) {
        if (enc == null) return 0;
        double angle = (enc.getVoltage() / enc.getMaxVoltage()) * 2 * Math.PI;
        angle *= ModuleConstants.kTurningMotorGearRatio;

        // 優先用 SharedPreferences 的 offset，沒有才用 Constants
        // 這裡簡化直接用 getGearedRad 當 offset（即套用目前的輪子位置）
        // 實際 Final = Geared - savedOffset
        float savedOffset = offsetPrefs.getFloat(
                "offset_" + getServoName(enc), Float.MAX_VALUE);
        if (savedOffset != Float.MAX_VALUE) {
            angle -= savedOffset;
        } else {
            angle -= DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad; // fallback
        }
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return Math.toDegrees(angle);
    }

    // ===== SharedPreferences 操作 =====
    private void saveOffset(String servoName, double rad) {
        offsetPrefs.edit().putFloat("offset_" + servoName, (float) rad).apply();
    }

    private void clearOffset(String servoName) {
        offsetPrefs.edit().remove("offset_" + servoName).apply();
    }

    private void clearAngle(String servoName) {
        anglePrefs.edit()
                .remove("swerve_angle_" + servoName)
                .remove("swerve_angle_" + servoName + "_raw")
                .apply();
    }

    private String getServoName(AnalogInput enc) {
        if (enc == flEncoder) return DriveConstants.kFrontLeftTurningMotorName;
        if (enc == frEncoder) return DriveConstants.kFrontRightTurningMotorName;
        if (enc == blEncoder) return DriveConstants.kBackLeftTurningMotorName;
        return DriveConstants.kBackRightTurningMotorName;
    }
}