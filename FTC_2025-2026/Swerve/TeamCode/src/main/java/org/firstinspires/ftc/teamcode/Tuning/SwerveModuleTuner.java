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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/**
 * Swerve 馬達方向測試程式
 *
 * 前置作業：先執行 "1. Encoder Offset Reader" 設定好 offset 值
 *
 * 功能：
 * - Init 階段：將所有輪子對齊到 0 度（車頭方向）
 * - Start 階段：測試驅動馬達方向
 *
 * 測試步驟：
 * 1. Init 時觀察輪子是否對齊到車頭方向
 *    - 如果沒有對齊，請先用 "1. Encoder Offset Reader" 調整 offset
 * 2. 按 Start 後測試驅動馬達：
 *    - A = 全部前進（輪子應該往車頭方向滾動）
 *    - B = 全部後退
 *    - X = 只有左側前進
 *    - Y = 只有右側前進
 * 3. 如果馬達方向錯誤，修改 Constants.java 的 kXXXDriveEncoderReversed
 */
@Config
@TeleOp(name = "2. Drive Motor Tester", group = "Tuning")
public class SwerveModuleTuner extends LinearOpMode {

    // Dashboard 可調參數
    public static double driveTestPower = 0.3;
    public static double turningP = Constants.ModuleConstants.kPTurning;
    public static double turningI = Constants.ModuleConstants.kITurning;
    public static double turningD = Constants.ModuleConstants.kDTurning;

    // 硬體
    private DcMotorEx flDrive, frDrive, blDrive, brDrive;
    private CRServo flTurn, frTurn, blTurn, brTurn;
    private AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;

    // PID Controllers
    private PIDController flPid, frPid, blPid, brPid;

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        initHardware();

        telemetry.addLine("=== Drive Motor Tester ===");
        telemetry.addLine("");
        telemetry.addLine("Init：輪子將對齊到 0°");
        telemetry.addLine("Start：測試驅動馬達方向");
        telemetry.addLine("");
        telemetry.addLine("如果輪子沒有正確對齊，");
        telemetry.addLine("請先執行 Encoder Offset Reader");
        telemetry.update();

        // ========== Init 階段：對齊輪子到 0 度 ==========
        while (!isStarted() && !isStopRequested()) {
            // 讀取角度
            double flDeg = getEncoderDegrees(flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);
            double frDeg = getEncoderDegrees(frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);
            double blDeg = getEncoderDegrees(blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);
            double brDeg = getEncoderDegrees(brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);

            // PID 控制到 0 度
            double flOut = calculateTurningOutput(flPid, flDeg, 0);
            double frOut = calculateTurningOutput(frPid, frDeg, 0);
            double blOut = calculateTurningOutput(blPid, blDeg, 0);
            double brOut = calculateTurningOutput(brPid, brDeg, 0);

            flTurn.setPower(flOut);
            frTurn.setPower(frOut);
            blTurn.setPower(blOut);
            brTurn.setPower(brOut);

            // 判斷是否對齊
            boolean flOk = Math.abs(flDeg) < 5;
            boolean frOk = Math.abs(frDeg) < 5;
            boolean blOk = Math.abs(blDeg) < 5;
            boolean brOk = Math.abs(brDeg) < 5;

            telemetry.addLine("===== Drive Motor Tester (Init) =====");
            telemetry.addLine("");
            telemetry.addLine("輪子正在對齊到 0°...");
            telemetry.addLine("");

            telemetry.addLine("===== 輪子角度 (目標: 0°) =====");
            telemetry.addData("FL", "%.1f° %s", flDeg, flOk ? "✓" : "");
            telemetry.addData("FR", "%.1f° %s", frDeg, frOk ? "✓" : "");
            telemetry.addData("BL", "%.1f° %s", blDeg, blOk ? "✓" : "");
            telemetry.addData("BR", "%.1f° %s", brDeg, brOk ? "✓" : "");
            telemetry.addLine("");

            if (flOk && frOk && blOk && brOk) {
                telemetry.addLine("★ 所有輪子已對齊！可以按 Start ★");
            } else {
                telemetry.addLine("等待輪子對齊中...");
            }

            // Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("FL", flDeg);
            packet.put("FR", frDeg);
            packet.put("BL", blDeg);
            packet.put("BR", brDeg);
            packet.put("target", 0);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
            idle();
        }

        // 重置驅動馬達 Encoder
        resetDriveEncoders();

        // ========== Start 階段：測試驅動馬達 ==========
        while (opModeIsActive()) {
            // 持續對齊輪子
            double flDeg = getEncoderDegrees(flEncoder, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);
            double frDeg = getEncoderDegrees(frEncoder, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);
            double blDeg = getEncoderDegrees(blEncoder, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);
            double brDeg = getEncoderDegrees(brEncoder, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);

            flTurn.setPower(calculateTurningOutput(flPid, flDeg, 0));
            frTurn.setPower(calculateTurningOutput(frPid, frDeg, 0));
            blTurn.setPower(calculateTurningOutput(blPid, blDeg, 0));
            brTurn.setPower(calculateTurningOutput(brPid, brDeg, 0));

            // 驅動馬達測試
            double power = 0;
            String status = "停止 (放開所有按鈕)";

            if (gamepad1.a) {
                power = driveTestPower;
                status = "▲ 全部前進 (A)";
                flDrive.setPower(power);
                frDrive.setPower(power);
                blDrive.setPower(power);
                brDrive.setPower(power);
            } else if (gamepad1.b) {
                power = -driveTestPower;
                status = "▼ 全部後退 (B)";
                flDrive.setPower(power);
                frDrive.setPower(power);
                blDrive.setPower(power);
                brDrive.setPower(power);
            } else if (gamepad1.x) {
                status = "◀ 只有左側前進 (X)";
                flDrive.setPower(driveTestPower);
                blDrive.setPower(driveTestPower);
                frDrive.setPower(0);
                brDrive.setPower(0);
            } else if (gamepad1.y) {
                status = "▶ 只有右側前進 (Y)";
                frDrive.setPower(driveTestPower);
                brDrive.setPower(driveTestPower);
                flDrive.setPower(0);
                blDrive.setPower(0);
            } else if (gamepad1.dpad_up) {
                status = "FL 單獨前進 (↑)";
                flDrive.setPower(driveTestPower);
                frDrive.setPower(0);
                blDrive.setPower(0);
                brDrive.setPower(0);
            } else if (gamepad1.dpad_right) {
                status = "FR 單獨前進 (→)";
                flDrive.setPower(0);
                frDrive.setPower(driveTestPower);
                blDrive.setPower(0);
                brDrive.setPower(0);
            } else if (gamepad1.dpad_left) {
                status = "BL 單獨前進 (←)";
                flDrive.setPower(0);
                frDrive.setPower(0);
                blDrive.setPower(driveTestPower);
                brDrive.setPower(0);
            } else if (gamepad1.dpad_down) {
                status = "BR 單獨前進 (↓)";
                flDrive.setPower(0);
                frDrive.setPower(0);
                blDrive.setPower(0);
                brDrive.setPower(driveTestPower);
            } else {
                flDrive.setPower(0);
                frDrive.setPower(0);
                blDrive.setPower(0);
                brDrive.setPower(0);
            }

            // 讀取 Encoder 位置
            int flPos = flDrive.getCurrentPosition();
            int frPos = frDrive.getCurrentPosition();
            int blPos = blDrive.getCurrentPosition();
            int brPos = brDrive.getCurrentPosition();

            telemetry.addLine("");

            telemetry.addData("狀態", status);
            telemetry.addData("測試功率", "%.1f%%", driveTestPower * 100);
            telemetry.addLine("");

            telemetry.addLine("══════ 輪子角度 ══════");
            telemetry.addData("FL", "%.1f°", flDeg);
            telemetry.addData("FR", "%.1f°", frDeg);
            telemetry.addData("BL", "%.1f°", blDeg);
            telemetry.addData("BR", "%.1f°", brDeg);
            telemetry.addLine("");

            telemetry.addLine("══════ 馬達 Encoder ══════");
            telemetry.addLine("(前進時數值應增加)");
            telemetry.addData("FL", "%d %s", flPos, flPos > 50 ? "↑" : (flPos < -50 ? "↓ 需反轉!" : ""));
            telemetry.addData("FR", "%d %s", frPos, frPos > 50 ? "↑" : (frPos < -50 ? "↓ 需反轉!" : ""));
            telemetry.addData("BL", "%d %s", blPos, blPos > 50 ? "↑" : (blPos < -50 ? "↓ 需反轉!" : ""));
            telemetry.addData("BR", "%d %s", brPos, brPos > 50 ? "↑" : (brPos < -50 ? "↓ 需反轉!" : ""));
            telemetry.addLine("");

            telemetry.addLine("══════ 控制說明 ══════");
            telemetry.addLine("A=全部前進 B=全部後退");
            telemetry.addLine("X=左側前進 Y=右側前進");
            telemetry.addLine("↑=FL ←=BL →=FR ↓=BR");

            // Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("FL_pos", flPos);
            packet.put("FR_pos", frPos);
            packet.put("BL_pos", blPos);
            packet.put("BR_pos", brPos);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }

        stopAll();
    }

    private void initHardware() {
        // 驅動馬達
        flDrive = hardwareMap.get(DcMotorEx.class, DriveConstants.kFrontLeftDriveMotorName);
        frDrive = hardwareMap.get(DcMotorEx.class, DriveConstants.kFrontRightDriveMotorName);
        blDrive = hardwareMap.get(DcMotorEx.class, DriveConstants.kBackLeftDriveMotorName);
        brDrive = hardwareMap.get(DcMotorEx.class, DriveConstants.kBackRightDriveMotorName);

        flDrive.setDirection(DriveConstants.kFrontLeftDriveEncoderReversed ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frDrive.setDirection(DriveConstants.kFrontRightDriveEncoderReversed ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        blDrive.setDirection(DriveConstants.kBackLeftDriveEncoderReversed ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        brDrive.setDirection(DriveConstants.kBackRightDriveEncoderReversed ?
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();

        // 轉向馬達
        flTurn = hardwareMap.get(CRServo.class, DriveConstants.kFrontLeftTurningMotorName);
        frTurn = hardwareMap.get(CRServo.class, DriveConstants.kFrontRightTurningMotorName);
        blTurn = hardwareMap.get(CRServo.class, DriveConstants.kBackLeftTurningMotorName);
        brTurn = hardwareMap.get(CRServo.class, DriveConstants.kBackRightTurningMotorName);

        flTurn.setDirection(DriveConstants.kFrontLeftTurningEncoderReversed ?
                CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        frTurn.setDirection(DriveConstants.kFrontRightTurningEncoderReversed ?
                CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        blTurn.setDirection(DriveConstants.kBackLeftTurningEncoderReversed ?
                CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        brTurn.setDirection(DriveConstants.kBackRightTurningEncoderReversed ?
                CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        // 絕對編碼器
        flEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontLeftAbsoluteEncoderName);
        frEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontRightAbsoluteEncoderName);
        blEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackLeftAbsoluteEncoderName);
        brEncoder = hardwareMap.get(AnalogInput.class, DriveConstants.kBackRightAbsoluteEncoderName);

        // PID
        flPid = new PIDController(turningP, turningI, turningD);
        frPid = new PIDController(turningP, turningI, turningD);
        blPid = new PIDController(turningP, turningI, turningD);
        brPid = new PIDController(turningP, turningI, turningD);
    }

    private void resetDriveEncoders() {
        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getEncoderDegrees(AnalogInput encoder, double offsetDeg) {
        double raw = (encoder.getVoltage() / encoder.getMaxVoltage()) * 360.0;
        double angle = raw - offsetDeg;
        // 環繞到 -90° ~ 90° 範圍
        while (angle > 90) angle -= 180;
        while (angle < -90) angle += 180;
        return angle;
    }

    private double calculateTurningOutput(PIDController pid, double currentDeg, double targetDeg) {
        pid.setPID(turningP, turningI, turningD);

        double error = targetDeg - currentDeg;
        // 環繞到 -90° ~ 90° 範圍
        while (error > 90) error -= 180;
        while (error < -90) error += 180;

        // 使用環繞處理後的誤差計算 PID 輸出，避免角度跳變
        double output = pid.calculate(0, Math.toRadians(error));
        output = Math.max(-0.8, Math.min(0.8, output));

        // 小誤差時降低輸出，避免抖動
        if (Math.abs(error) < 3) output *= 0.5;

        return output;
    }

    private void stopAll() {
        flDrive.setPower(0);
        frDrive.setPower(0);
        blDrive.setPower(0);
        brDrive.setPower(0);
        flTurn.setPower(0);
        frTurn.setPower(0);
        blTurn.setPower(0);
        brTurn.setPower(0);
    }
}
