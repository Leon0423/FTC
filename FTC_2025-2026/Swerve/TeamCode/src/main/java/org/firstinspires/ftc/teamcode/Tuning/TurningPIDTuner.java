package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * Turning PID 調整 OpMode - 四輪同時測試
 *
 * 使用方法：
 * 1. 連接 FTC Dashboard (http://192.168.43.1:8080/dash)
 * 2. 在 Config 調整 TuningConfig.turningP/I/D
 * 3. 在 Graph 觀察：target, FL, FR, BL, BR
 */
@Config
@TeleOp(name = "Turning PID Tuner", group = "Tuning")
public class TurningPIDTuner extends LinearOpMode {

    // Dashboard 可調參數 (測試專用)
    public static double testPeriodSec = 2.0;      // 測試週期 (秒)
    public static double testAmplitudeDeg = 80;    // 測試振幅 (度)

    private SwerveSubsystem swerve;
    private FtcDashboard dashboard;

    // 四個獨立的 PID Controller 用於直接控制
    private PIDController flPID, frPID, blPID, brPID;

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new SwerveSubsystem(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // 初始化 PID Controllers
        flPID = new PIDController(TuningConfig.turningP, TuningConfig.turningI, TuningConfig.turningD);
        frPID = new PIDController(TuningConfig.turningP, TuningConfig.turningI, TuningConfig.turningD);
        blPID = new PIDController(TuningConfig.turningP, TuningConfig.turningI, TuningConfig.turningD);
        brPID = new PIDController(TuningConfig.turningP, TuningConfig.turningI, TuningConfig.turningD);

        telemetry.addLine("=== Turning PID Tuner (4輪同時) ===");
        telemetry.addLine("Dashboard Config 調整: TuningConfig.turningP/I/D");
        telemetry.addLine("Dashboard Graph 觀察: target, FL, FR, BL, BR");
        telemetry.update();

        waitForStart();

        double startTime = getRuntime();

        while (opModeIsActive()) {
            // 更新 PID 參數 (即時調整)
            flPID.setPID(TuningConfig.turningP, TuningConfig.turningI, TuningConfig.turningD);
            frPID.setPID(TuningConfig.turningP, TuningConfig.turningI, TuningConfig.turningD);
            blPID.setPID(TuningConfig.turningP, TuningConfig.turningI, TuningConfig.turningD);
            brPID.setPID(TuningConfig.turningP, TuningConfig.turningI, TuningConfig.turningD);

            // 產生正弦波目標角度
            double elapsed = getRuntime() - startTime;
            double targetRad = Math.toRadians(testAmplitudeDeg) * Math.sin(2 * Math.PI * elapsed / testPeriodSec);

            // 取得各輪模組
            SwerveModule fl = swerve.getFrontLeft();
            SwerveModule fr = swerve.getFrontRight();
            SwerveModule bl = swerve.getBackLeft();
            SwerveModule br = swerve.getBackRight();

            // 直接讀取 encoder 角度（不再跳變過濾）
            double flAngle = fl.getTurningPosition();
            double frAngle = fr.getTurningPosition();
            double blAngle = bl.getTurningPosition();
            double brAngle = br.getTurningPosition();

            // 直接對每個輪子應用 PID 控制，不經過 setDesiredState 的 optimize
            double flOutput = applyTurningPID(flPID, flAngle, targetRad);
            double frOutput = applyTurningPID(frPID, frAngle, targetRad);
            double blOutput = applyTurningPID(blPID, blAngle, targetRad);
            double brOutput = applyTurningPID(brPID, brAngle, targetRad);

            // 實際設定轉向馬達功率
            fl.setTurningPower(flOutput);
            fr.setTurningPower(frOutput);
            bl.setTurningPower(blOutput);
            br.setTurningPower(brOutput);

            // 計算誤差 (用於顯示)
            double flError = Math.toDegrees(normalizeAngle(targetRad - flAngle));
            double frError = Math.toDegrees(normalizeAngle(targetRad - frAngle));
            double blError = Math.toDegrees(normalizeAngle(targetRad - blAngle));
            double brError = Math.toDegrees(normalizeAngle(targetRad - brAngle));

            // 圖表數據 (目標、當前角度、誤差)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target", Math.toDegrees(targetRad));
            packet.put("FL", Math.toDegrees(flAngle));
            packet.put("FR", Math.toDegrees(frAngle));
            packet.put("BL", Math.toDegrees(blAngle));
            packet.put("BR", Math.toDegrees(brAngle));
            packet.put("FL_error", flError);
            packet.put("FR_error", frError);
            packet.put("BL_error", blError);
            packet.put("BR_error", brError);
            dashboard.sendTelemetryPacket(packet);

            // 文字顯示
            telemetry.addData("P", "%.4f", TuningConfig.turningP);
            telemetry.addData("I", "%.4f", TuningConfig.turningI);
            telemetry.addData("D", "%.4f", TuningConfig.turningD);
            telemetry.addLine("");
            telemetry.addData("Target", "%.1f°", Math.toDegrees(targetRad));
            telemetry.addLine("");
            telemetry.addData("FL Angle", "%.1f° (err: %.1f°)", Math.toDegrees(flAngle), flError);
            telemetry.addData("FR Angle", "%.1f° (err: %.1f°)", Math.toDegrees(frAngle), frError);
            telemetry.addData("BL Angle", "%.1f° (err: %.1f°)", Math.toDegrees(blAngle), blError);
            telemetry.addData("BR Angle", "%.1f° (err: %.1f°)", Math.toDegrees(brAngle), brError);
            telemetry.update();
        }

        swerve.stopModules();
    }

    /**
     * 直接應用 PID 控制到轉向馬達，繞過 SwerveModuleState.optimize()
     */
    private double applyTurningPID(PIDController pid, double currentRad, double targetRad) {
        double error = normalizeAngle(targetRad - currentRad);
        double errorDeg = Math.abs(Math.toDegrees(error));

        double output = pid.calculate(0, error) * TuningConfig.turningOutputScale;
        output = Math.max(-1.0, Math.min(1.0, output));

        if (errorDeg < TuningConfig.deadbandDeg) {
            output = 0; // only deadband now
        }

        return output;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
