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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * Turning PID 調整 OpMode - 四輪同時測試
 *
 * ★ 使用與 Swerve_Control 完全相同的控制流程 ★
 * 透過 SwerveModuleState.optimize() 控制
 * 這樣調出來的 PID 值才是真正在實際運作時使用的值
 *
 * ★ 所有參數獨立於 TuningConfig，在 Dashboard 的 TurningPIDTuner 區塊調整 ★
 *
 * 使用方法：
 * 1. 連接 FTC Dashboard (http://192.168.43.1:8080/dash)
 * 2. 在 Config > TurningPIDTuner 調整 P/I/D 參數
 * 3. 在 Graph 觀察：target, FL, FR, BL, BR
 */
@Config
@TeleOp(name = "4. Turning PID Tuner", group = "Tuning")
public class _4_TurningPIDTuner extends LinearOpMode {

    // ===== Dashboard 可調參數 (獨立於 TuningConfig) =====
    // 1. Turning PID 參數
    public static double _1a_turningP = Constants.ModuleConstants.kPTurning;
    public static double _1b_turningI = Constants.ModuleConstants.kITurning;
    public static double _1c_turningD = Constants.ModuleConstants.kDTurning;
    public static double _1d_outputScale = Constants.ModuleConstants.kTurningOutputScale;
    public static double _1e_deadbandDeg = Constants.ModuleConstants.kTurningDeadbandDeg;

    // 2. 測試參數
    public static double _2a_testPeriodSec = 2.0;       // 測試週期 (秒)
    public static double _2b_testAmplitudeDeg = 45.0;   // 測試振幅 (度)

    private SwerveSubsystem swerve;
    private FtcDashboard dashboard;

    // 四個獨立的 PID Controller
    private PIDController flPID, frPID, blPID, brPID;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new SwerveSubsystem(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // 初始化 PID Controllers
        flPID = new PIDController(_1a_turningP, _1b_turningI, _1c_turningD);
        frPID = new PIDController(_1a_turningP, _1b_turningI, _1c_turningD);
        blPID = new PIDController(_1a_turningP, _1b_turningI, _1c_turningD);
        brPID = new PIDController(_1a_turningP, _1b_turningI, _1c_turningD);

        telemetry.addLine("=== Turning PID Tuner (4輪同時) ===");
        telemetry.addLine("★ 參數獨立於 Swerve_Control ★");
        telemetry.addLine("Dashboard Config > TurningPIDTuner 調整參數");
        telemetry.addLine("Dashboard Graph 觀察: target, FL, FR, BL, BR");
        telemetry.update();

        waitForStart();

        double startTime = getRuntime();

        while (opModeIsActive()) {
            // 即時更新 PID 參數
            flPID.setPID(_1a_turningP, _1b_turningI, _1c_turningD);
            frPID.setPID(_1a_turningP, _1b_turningI, _1c_turningD);
            blPID.setPID(_1a_turningP, _1b_turningI, _1c_turningD);
            brPID.setPID(_1a_turningP, _1b_turningI, _1c_turningD);

            // 產生正弦波目標角度
            double elapsed = getRuntime() - startTime;
            double targetRad = Math.toRadians(_2b_testAmplitudeDeg) * Math.sin(2 * Math.PI * elapsed / _2a_testPeriodSec);

            // 取得各輪模組
            SwerveModule fl = swerve.getFrontLeft();
            SwerveModule fr = swerve.getFrontRight();
            SwerveModule bl = swerve.getBackLeft();
            SwerveModule br = swerve.getBackRight();

            // 讀取實際角度
            double flAngle = fl.getAbsoluteEncoderRad();
            double frAngle = fr.getAbsoluteEncoderRad();
            double blAngle = bl.getAbsoluteEncoderRad();
            double brAngle = br.getAbsoluteEncoderRad();

            // ★ 使用與 SwerveModule.setDesiredState 相同的控制邏輯 ★
            // 建立 SwerveModuleState 並執行 optimize
            SwerveModuleState targetState = new SwerveModuleState(1.0, new Rotation2d(targetRad));

            // FL
            SwerveModuleState flOptimized = SwerveModuleState.optimize(targetState, new Rotation2d(flAngle));
            double flError = normalizeAngle(flOptimized.angle.getRadians() - flAngle);
            double flOutput = applyPID(flPID, flError);
            fl.setTurningPower(flOutput);

            // FR
            SwerveModuleState frOptimized = SwerveModuleState.optimize(targetState, new Rotation2d(frAngle));
            double frError = normalizeAngle(frOptimized.angle.getRadians() - frAngle);
            double frOutput = applyPID(frPID, frError);
            fr.setTurningPower(frOutput);

            // BL
            SwerveModuleState blOptimized = SwerveModuleState.optimize(targetState, new Rotation2d(blAngle));
            double blError = normalizeAngle(blOptimized.angle.getRadians() - blAngle);
            double blOutput = applyPID(blPID, blError);
            bl.setTurningPower(blOutput);

            // BR
            SwerveModuleState brOptimized = SwerveModuleState.optimize(targetState, new Rotation2d(brAngle));
            double brError = normalizeAngle(brOptimized.angle.getRadians() - brAngle);
            double brOutput = applyPID(brPID, brError);
            br.setTurningPower(brOutput);

            // 圖表數據
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target", Math.toDegrees(targetRad));
            packet.put("FL", Math.toDegrees(flAngle));
            packet.put("FR", Math.toDegrees(frAngle));
            packet.put("BL", Math.toDegrees(blAngle));
            packet.put("BR", Math.toDegrees(brAngle));
            packet.put("FL_error", Math.toDegrees(flError));
            packet.put("FR_error", Math.toDegrees(frError));
            packet.put("BL_error", Math.toDegrees(blError));
            packet.put("BR_error", Math.toDegrees(brError));
            packet.put("FL_output", flOutput);
            packet.put("FR_output", frOutput);
            packet.put("BL_output", blOutput);
            packet.put("BR_output", brOutput);
            dashboard.sendTelemetryPacket(packet);

            // 文字顯示
            telemetry.addData("P", "%.4f", _1a_turningP);
            telemetry.addData("I", "%.4f", _1b_turningI);
            telemetry.addData("D", "%.4f", _1c_turningD);
            telemetry.addData("Output Scale", "%.2f", _1d_outputScale);
            telemetry.addData("Deadband", "%.1f°", _1e_deadbandDeg);
            telemetry.addLine("");
            telemetry.addData("Target", "%.1f°", Math.toDegrees(targetRad));
            telemetry.addData("Test Period", "%.1f sec", _2a_testPeriodSec);
            telemetry.addData("Test Amplitude", "%.1f°", _2b_testAmplitudeDeg);
            telemetry.addLine("");
            telemetry.addData("FL", "%.1f° (err: %.1f°, out: %.2f)",
                    Math.toDegrees(flAngle), Math.toDegrees(flError), flOutput);
            telemetry.addData("FR", "%.1f° (err: %.1f°, out: %.2f)",
                    Math.toDegrees(frAngle), Math.toDegrees(frError), frOutput);
            telemetry.addData("BL", "%.1f° (err: %.1f°, out: %.2f)",
                    Math.toDegrees(blAngle), Math.toDegrees(blError), blOutput);
            telemetry.addData("BR", "%.1f° (err: %.1f°, out: %.2f)",
                    Math.toDegrees(brAngle), Math.toDegrees(brError), brOutput);
            telemetry.update();
        }

        swerve.stopModules();
    }

    /**
     * 應用 PID 控制（與 SwerveModule.setDesiredState 相同邏輯）
     */
    private double applyPID(PIDController pid, double error) {
        double errorDeg = Math.abs(Math.toDegrees(error));

        double output = pid.calculate(0, error) * _1d_outputScale;
        output = Math.max(-1.0, Math.min(1.0, output));

        if (errorDeg < _1e_deadbandDeg) {
            output = 0;
        }

        return output;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
