package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * FR 驅動馬達獨立測試 OpMode
 * 用於診斷 FR (前右) 驅動馬達力矩過大轉不動的問題
 * 
 * 操作方式：
 * - 左搖桿 Y 軸：控制馬達功率 (上推 = 正方向, 下推 = 反方向)
 * - A 鍵：重置編碼器
 * - B 鍵：執行 50% 功率測試 5 秒
 * - X 鍵：執行漸進功率測試 (10% → 100%)
 */
@TeleOp(name = "FR_DriveMotor_Test", group = "Diagnostic")
public class FRMotorTest extends LinearOpMode {
    
    private DcMotorEx frDriveMotor;
    private double lastPower = 0;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        // 嘗試取得 FR 驅動馬達
        try {
            frDriveMotor = hardwareMap.get(DcMotorEx.class, "FL");
        } catch (Exception e) {
            telemetry.addData("ERROR", "找不到 'FR' 驅動馬達");
            telemetry.addData("信息", "檢查 hardwareMap 配置");
            telemetry.addData("應該的名稱", "FR");
            telemetry.update();
            requestOpModeStop();
            return;
        }
        
        // 配置馬達
        frDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frDriveMotor.setPower(0);
        
        telemetry.addData("Status", "✓ FR 驅動馬達測試已初始化");
        telemetry.addData("提示", "按 START 開始測試");
        telemetry.update();
        
        waitForStart();
        
        telemetry.clear();
        
        while (opModeIsActive()) {
            
            // 從遊戲控制器讀取搖桿輸入
            double joystickPower = -gamepad1.left_stick_y;  // 上推 = 正功率
            
            // 應用死區
            if (Math.abs(joystickPower) < 0.05) {
                joystickPower = 0;
            }
            
            // 限制功率範圍
            joystickPower = Math.max(-1.0, Math.min(1.0, joystickPower));
            
            // 設定馬達功率
            frDriveMotor.setPower(joystickPower);
            lastPower = joystickPower;
            
            // A 鍵：重置編碼器
            if (gamepad1.a) {
                frDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("ACTION", "✓ 編碼器已重置");
            }
            
            // B 鍵：執行 50% 功率測試
            if (gamepad1.b) {
                telemetry.addData("ACTION", "執行 50% 功率測試 (5 秒)...");
                telemetry.update();
                frDriveMotor.setPower(0.5);
                sleep(5000);
                frDriveMotor.setPower(0);
                telemetry.addData("ACTION", "✓ 測試完成");
            }
            
            // X 鍵：漸進功率測試 (診斷最小起動功率)
            if (gamepad1.x) {
                telemetry.addData("ACTION", "執行漸進功率測試...");
                telemetry.update();
                
                for (double testPower = 0.05; testPower <= 1.0; testPower += 0.05) {
                    telemetry.addData("测试功率", "%.0f%%", testPower * 100);
                    telemetry.addData("RPM", "%.0f", getRPM(frDriveMotor));
                    telemetry.update();
                    
                    frDriveMotor.setPower(testPower);
                    sleep(500);
                }
                frDriveMotor.setPower(0);
            }
            
            // 顯示馬達狀態
            telemetry.addData("=== FR 驅動馬達狀態 ===", "");
            telemetry.addData("功率輸入", "%.2f (%.0f%%)", joystickPower, joystickPower * 100);
            telemetry.addData("當前功率", "%.2f", frDriveMotor.getPower());
            telemetry.addData("編碼器位置", "%d ticks", frDriveMotor.getCurrentPosition());
            telemetry.addData("轉速 (RPM)", "%.0f", getRPM(frDriveMotor));
            telemetry.addData("轉速 (ticks/sec)", "%.0f", frDriveMotor.getVelocity());
            
            // 馬達狀態診斷
            telemetry.addData("", "");
            telemetry.addData("=== 診斷信息 ===", "");
            
            if (joystickPower > 0.1 && frDriveMotor.getVelocity() == 0) {
                telemetry.addData("⚠️ 警告", "有功率 (>10%) 但轉速為 0 - FR馬達卡住!");
            } else if (joystickPower > 0.3 && getRPM(frDriveMotor) < 50) {
                telemetry.addData("⚠️ 警告", "功率充足但轉速異常低 - 可能有問題");
            } else if (joystickPower == 0) {
                telemetry.addData("✓ 狀態", "待命 (功率為 0)");
            } else {
                telemetry.addData("✓ 狀態", "正常轉動");
            }
            
            telemetry.addData("", "");
            telemetry.addData("=== 控制方式 ===", "");
            telemetry.addData("左搖桿 Y", "控制功率 ±100%");
            telemetry.addData("A 鍵", "重置編碼器");
            telemetry.addData("B 鍵", "50% 功率測試 (5秒)");
            telemetry.addData("X 鍵", "漸進功率測試 (5%-100%)");
            
            telemetry.update();
        }
        
        // 停止馬達
        frDriveMotor.setPower(0);
    }
    
    /**
     * 將馬達速度轉換為 RPM
     * goBILDA 馬達: 28 ticks/revolution
     */
    private double getRPM(DcMotorEx motor) {
        double ticksPerSec = motor.getVelocity();
        return (ticksPerSec / 28.0) * 60.0;
    }
}


