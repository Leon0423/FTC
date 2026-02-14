package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/**
 * 編碼器 Offset 讀取程式 (AXON Servo 版本)
 *
 * 功能：
 * - 讀取並顯示 AXON Servo 絕對編碼器的原始角度
 * - 可以手動轉動輪子，觀察角度變化
 *
 * 使用步驟：
 * 1. 運行此程式
 * 2. 手動將所有輪子轉到朝向車頭方向（齒輪標記對齊）
 * 3. 記錄每個輪子顯示的 "Raw Angle" 值
 * 4. 將這些值填入 Constants.java 的 kXXXDriveAbsoluteEncoderOffsetDeg
 *
 * 例如：
 * FL 顯示 45.2° → kFrontLeftDriveAbsoluteEncoderOffsetDeg = 45.2
 */
@Config
@TeleOp(name = "1. Encoder Offset Reader", group = "Tuning")
public class EncoderOffsetReader extends LinearOpMode {

    // AXON Servo 的 CRServo（需要初始化才能讓編碼器通電）
    private CRServo flServo, frServo, blServo, brServo;

    // AXON Servo 的絕對編碼器
    private AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // 嘗試初始化硬體
        boolean initSuccess = true;
        StringBuilder errorMsg = new StringBuilder();

        // 初始化 CRServo（讓 AXON Servo 通電）
        try {
            flServo = hardwareMap.get(CRServo.class, DriveConstants.kFrontLeftTurningMotorName);
            flServo.setPower(0); // 設定 0 功率但讓它通電
        } catch (Exception e) {
            initSuccess = false;
            errorMsg.append("FL Servo: ").append(DriveConstants.kFrontLeftTurningMotorName).append(" 找不到!\n");
        }

        try {
            frServo = hardwareMap.get(CRServo.class, DriveConstants.kFrontRightTurningMotorName);
            frServo.setPower(0);
        } catch (Exception e) {
            initSuccess = false;
            errorMsg.append("FR Servo: ").append(DriveConstants.kFrontRightTurningMotorName).append(" 找不到!\n");
        }

        try {
            blServo = hardwareMap.get(CRServo.class, DriveConstants.kBackLeftTurningMotorName);
            blServo.setPower(0);
        } catch (Exception e) {
            initSuccess = false;
            errorMsg.append("BL Servo: ").append(DriveConstants.kBackLeftTurningMotorName).append(" 找不到!\n");
        }

        try {
            brServo = hardwareMap.get(CRServo.class, DriveConstants.kBackRightTurningMotorName);
            brServo.setPower(0);
        } catch (Exception e) {
            initSuccess = false;
            errorMsg.append("BR Servo: ").append(DriveConstants.kBackRightTurningMotorName).append(" 找不到!\n");
        }

        // 初始化絕對編碼器
        String flEncoderName = DriveConstants.kFrontLeftAbsoluteEncoderName;
        String frEncoderName = DriveConstants.kFrontRightAbsoluteEncoderName;
        String blEncoderName = DriveConstants.kBackLeftAbsoluteEncoderName;
        String brEncoderName = DriveConstants.kBackRightAbsoluteEncoderName;

        try {
            flEncoder = hardwareMap.get(AnalogInput.class, flEncoderName);
        } catch (Exception e) {
            initSuccess = false;
            errorMsg.append("FL Encoder: ").append(flEncoderName).append(" 找不到!\n");
        }

        try {
            frEncoder = hardwareMap.get(AnalogInput.class, frEncoderName);
        } catch (Exception e) {
            initSuccess = false;
            errorMsg.append("FR Encoder: ").append(frEncoderName).append(" 找不到!\n");
        }

        try {
            blEncoder = hardwareMap.get(AnalogInput.class, blEncoderName);
        } catch (Exception e) {
            initSuccess = false;
            errorMsg.append("BL Encoder: ").append(blEncoderName).append(" 找不到!\n");
        }

        try {
            brEncoder = hardwareMap.get(AnalogInput.class, brEncoderName);
        } catch (Exception e) {
            initSuccess = false;
            errorMsg.append("BR Encoder: ").append(brEncoderName).append(" 找不到!\n");
        }

        // Init 階段顯示硬體狀態並即時讀取
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("══════ Encoder Offset Reader ══════");
            telemetry.addLine("(AXON Servo 版本)");
            telemetry.addLine("");

            if (!initSuccess) {
                telemetry.addLine("⚠️ 硬體初始化失敗！");
                telemetry.addLine("");
                telemetry.addLine(errorMsg.toString());
                telemetry.addLine("");
            }

            telemetry.addLine("══════ 硬體名稱 ══════");
            telemetry.addData("FL Servo", DriveConstants.kFrontLeftTurningMotorName);
            telemetry.addData("FR Servo", DriveConstants.kFrontRightTurningMotorName);
            telemetry.addData("BL Servo", DriveConstants.kBackLeftTurningMotorName);
            telemetry.addData("BR Servo", DriveConstants.kBackRightTurningMotorName);
            telemetry.addLine("");
            telemetry.addData("FL Encoder", flEncoderName);
            telemetry.addData("FR Encoder", frEncoderName);
            telemetry.addData("BL Encoder", blEncoderName);
            telemetry.addData("BR Encoder", brEncoderName);
            telemetry.addLine("");

            if (initSuccess) {
                // 在 Init 階段也顯示即時數據
                telemetry.addLine("══════ 即時讀數 ══════");
                telemetry.addData("FL", "%.3fV → %.1f°", flEncoder.getVoltage(), getOffsetDegrees(flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg));
                telemetry.addData("FR", "%.3fV → %.1f°", frEncoder.getVoltage(), getOffsetDegrees(frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg));
                telemetry.addData("BL", "%.3fV → %.1f°", blEncoder.getVoltage(), getOffsetDegrees(blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg));
                telemetry.addData("BR", "%.3fV → %.1f°", brEncoder.getVoltage(), getOffsetDegrees(brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg));
                telemetry.addLine("");
                telemetry.addData("MaxVoltage", "%.3fV", flEncoder.getMaxVoltage());
                telemetry.addLine("");
                telemetry.addLine("手動轉動輪子到車頭方向");
                telemetry.addLine("記錄上面的角度值");
            }

            telemetry.update();
            idle();
        }

        if (!initSuccess) {
            return;
        }

        // Start 後的主迴圈
        while (opModeIsActive()) {
            // 讀取原始角度（0-360度）
            double flRaw = getRawDegrees(flEncoder);
            double frRaw = getRawDegrees(frEncoder);
            double blRaw = getRawDegrees(blEncoder);
            double brRaw = getRawDegrees(brEncoder);

            // 讀取電壓
            double flVolt = flEncoder.getVoltage();
            double frVolt = frEncoder.getVoltage();
            double blVolt = blEncoder.getVoltage();
            double brVolt = brEncoder.getVoltage();

            // 計算套用目前 offset 後的角度
            double flCurrent = getOffsetDegrees(flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);
            double frCurrent = getOffsetDegrees(frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);
            double blCurrent = getOffsetDegrees(blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);
            double brCurrent = getOffsetDegrees(brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);

            // 顯示資訊
            telemetry.addLine("══════ Encoder Offset Reader ══════");
            telemetry.addLine("");
            telemetry.addLine("手動轉動輪子到車頭方向後");
            telemetry.addLine("複製 Raw Angle 到 Constants.java");
            telemetry.addLine("");

            telemetry.addLine("══════ 複製這些值 ══════");
            telemetry.addData("FL OffsetDeg", "%.1f", flRaw);
            telemetry.addData("FR OffsetDeg", "%.1f", frRaw);
            telemetry.addData("BL OffsetDeg", "%.1f", blRaw);
            telemetry.addData("BR OffsetDeg", "%.1f", brRaw);
            telemetry.addLine("");

            telemetry.addLine("══════ 詳細資訊 ══════");
            telemetry.addData("FL", "Raw:%.1f° | V:%.3f | Cur:%.1f°", flRaw, flVolt, flCurrent);
            telemetry.addData("FR", "Raw:%.1f° | V:%.3f | Cur:%.1f°", frRaw, frVolt, frCurrent);
            telemetry.addData("BL", "Raw:%.1f° | V:%.3f | Cur:%.1f°", blRaw, blVolt, blCurrent);
            telemetry.addData("BR", "Raw:%.1f° | V:%.3f | Cur:%.1f°", brRaw, brVolt, brCurrent);

            // Dashboard 圖表
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("FL_raw", flRaw);
            packet.put("FR_raw", frRaw);
            packet.put("BL_raw", blRaw);
            packet.put("BR_raw", brRaw);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }

    private double getRawDegrees(AnalogInput encoder) {
        if (encoder == null) return 0;
        double maxV = encoder.getMaxVoltage();
        if (maxV == 0) return 0;
        return (encoder.getVoltage() / maxV) * 360.0;
    }

    private double getOffsetDegrees(AnalogInput encoder, double offsetDeg) {
        double raw = getRawDegrees(encoder);
        double angle = raw - offsetDeg;
        // 先環繞到 -180° ~ 180° 範圍
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        // 再環繞到 -90° ~ 90° 範圍
        while (angle > 90) angle -= 180;
        while (angle < -90) angle += 180;
        return angle;
    }
}

