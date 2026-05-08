package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants.ModuleConstants;


import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

@TeleOp(name = "1b. Turning Encoder Reverse", group = "Tuning")
public class _1b_TurningEncoderReverse extends LinearOpMode {

    private CRServo frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn;
    private AnalogInput flAbs, frAbs, blAbs, brAbs;

    private final ElapsedTime pulseTimer = new ElapsedTime();
    private boolean pulseActive = false;
    private boolean lastAPressed = false;
    private boolean lastRb = false;
    private boolean lastLb = false;
    private double pulsePower = 0.3;

    // 記錄每次 pulse 前的角度
    private double flAngleBefore, frAngleBefore, blAngleBefore, brAngleBefore;

    // 記錄 pulse 後的角度變化結論
    private String flVerdict = "---";
    private String frVerdict = "---";
    private String blVerdict = "---";
    private String brVerdict = "---";

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftTurn  = hardwareMap.get(CRServo.class, DriveConstants.kFrontLeftTurningMotorName);
        frontRightTurn = hardwareMap.get(CRServo.class, DriveConstants.kFrontRightTurningMotorName);
        backLeftTurn   = hardwareMap.get(CRServo.class, DriveConstants.kBackLeftTurningMotorName);
        backRightTurn  = hardwareMap.get(CRServo.class, DriveConstants.kBackRightTurningMotorName);

        flAbs = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontLeftAbsoluteEncoderName);
        frAbs = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontRightAbsoluteEncoderName);
        blAbs = hardwareMap.get(AnalogInput.class, DriveConstants.kBackLeftAbsoluteEncoderName);
        brAbs = hardwareMap.get(AnalogInput.class, DriveConstants.kBackRightAbsoluteEncoderName);

        frontLeftTurn.setDirection(
                DriveConstants.kFrontLeftTurningEncoderReversed ?
                        DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        frontRightTurn.setDirection(
                DriveConstants.kFrontRightTurningEncoderReversed ?
                        DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        backLeftTurn.setDirection(
                DriveConstants.kBackLeftTurningEncoderReversed ?
                        DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        backRightTurn.setDirection(
                DriveConstants.kBackRightTurningEncoderReversed ?
                        DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);


        waitForStart();
        pulseTimer.reset();

        while (opModeIsActive()) {

            // ===== 調整 pulse 功率 =====
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            if (rb && !lastRb) pulsePower = Math.min( 1.0, pulsePower + 0.1);
            if (lb && !lastLb) pulsePower = Math.max(-1.0, pulsePower - 0.1);
            lastRb = rb;
            lastLb = lb;

            boolean aPressed = gamepad1.a;

            // ===== A 按下：記錄 pulse 前角度 =====
            if (aPressed && !lastAPressed) {
                flAngleBefore = rawDeg(flAbs);
                frAngleBefore = rawDeg(frAbs);
                blAngleBefore = rawDeg(blAbs);
                brAngleBefore = rawDeg(brAbs);
                flVerdict = "---";
                frVerdict = "---";
                blVerdict = "---";
                brVerdict = "---";
                pulseActive = true;
                pulseTimer.reset();
            }
            lastAPressed = aPressed;

            // ===== Pulse 結束：計算角度變化，給出結論 =====
            double power = 0;
            if (pulseActive) {
                power = pulsePower;
                if (pulseTimer.milliseconds() > 300) {
                    pulseActive = false;
                    power = 0;

                    flVerdict = verdict(flAngleBefore, rawDeg(flAbs), pulsePower,
                            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
                    frVerdict = verdict(frAngleBefore, rawDeg(frAbs), pulsePower,
                            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
                    blVerdict = verdict(blAngleBefore, rawDeg(blAbs), pulsePower,
                            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
                    brVerdict = verdict(brAngleBefore, rawDeg(brAbs), pulsePower,
                            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
                }
            }

            frontLeftTurn.setPower(power);
            frontRightTurn.setPower(power);
            backLeftTurn.setPower(power);
            backRightTurn.setPower(power);

            // ===== Telemetry =====
            telemetry.addLine("════ 操作說明 ════");
            telemetry.addData("A 鍵", "發送 pulse（觀察角度變化）");
            telemetry.addData("LB / RB", "調整功率 ±10%%");
            telemetry.addData("Pulse 功率", "%.0f%%  (%s方向)",
                    pulsePower * 100,
                    pulsePower >= 0 ? "正" : "負");
            telemetry.addLine("");

            telemetry.addLine("════ 即時角度（Raw 0~360°）════");
            telemetry.addData("FL", "%.1f°", rawDeg(flAbs));
            telemetry.addData("FR", "%.1f°", rawDeg(frAbs));
            telemetry.addData("BL", "%.1f°", rawDeg(blAbs));
            telemetry.addData("BR", "%.1f°", rawDeg(brAbs));
            telemetry.addLine("");

            telemetry.addLine("════ Pulse 結論（按 A 後更新）════");
            telemetry.addLine("判斷基準：正 power 時，PID angle delta 應為正");
            telemetry.addLine("若 PID delta 方向相反，優先切換 AbsoluteEncoderReversed");
            telemetry.addData("FL", flVerdict);
            telemetry.addData("FR", frVerdict);
            telemetry.addData("BL", blVerdict);
            telemetry.addData("BR", brVerdict);
            telemetry.addLine("");

            telemetry.addLine("════ 目前 Constants 設定 ════");
            telemetry.addData("Turning motor reversed", "FL:%s FR:%s BL:%s BR:%s",
                    DriveConstants.kFrontLeftTurningEncoderReversed,
                    DriveConstants.kFrontRightTurningEncoderReversed,
                    DriveConstants.kBackLeftTurningEncoderReversed,
                    DriveConstants.kBackRightTurningEncoderReversed);
            telemetry.addData("Absolute encoder reversed", "FL:%s FR:%s BL:%s BR:%s",
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
                    DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

            telemetry.update();
        }

        frontLeftTurn.setPower(0);
        frontRightTurn.setPower(0);
        backLeftTurn.setPower(0);
        backRightTurn.setPower(0);
    }

    /**
     * 計算角度變化，處理 0/360° 邊界跳躍
     * @return 結論字串，說明需不需要 Reverse
     */
    private String verdict(double before, double after, double power, boolean absoluteEncoderReversed) {
     *double rawDelta = wrapDeltaDeg(after - before);
     *double pidDelta = rawDelta
                * * (absoluteEncoderReversed ? -1.0 : 1.0)
                * * ModuleConstants.kTurningMotorGearRatio;


        if (Math.abs(pidDelta) < 0.4) {
            return "變化太小，加大功率再試";
        }

        boolean pidAngleIncreased = pidDelta > 0;
        boolean powerPositive  = power > 0;

        boolean needFlipAbsolute = (powerPositive && !pidAngleIncreased)
                || (!powerPositive && pidAngleIncreased);

        return String.format("raw:%+.1f° pid:%+.1f° -> %s",
                rawDelta,
                pidDelta,
                needFlipAbsolute ? "切換 AbsoluteReverse" : "OK");
    }

    private double rawDeg(AnalogInput input) {
        return (input.getVoltage() / input.getMaxVoltage()) * 360.0;
    }
    private double wrapDeltaDeg(double delta) {
        if (delta >  180) delta -= 360;
        if (delta < -180) delta += 360;
        return delta;
    }
}