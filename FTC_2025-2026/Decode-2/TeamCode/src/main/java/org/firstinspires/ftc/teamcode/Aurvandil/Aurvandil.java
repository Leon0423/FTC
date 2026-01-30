package org.firstinspires.ftc.teamcode.Aurvandil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Aurvandil 機器人主控程式
 * 包含底盤驅動、射球機構、吸球機構的完整控制
 */
@TeleOp(name = "Aurvandil", group = "Aurvandil")
public class Aurvandil extends LinearOpMode {

    // ===== 底盤馬達 =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;  // 四輪驅動馬達
    private DcMotor[] driveMotors;  // 馬達陣列，方便批次操作

    // ===== 射球機構 =====
    private DcMotorEx shooterMotor;  // 射球馬達（使用Ex版本以支援速度控制）
    private DcMotor intake1, intake2;  // intake1: 吸球馬達, intake2: 送球馬達

    // ===== 常數設定 =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0;  // 編碼器每轉脈衝數
    private static final double HIGH_RPM = 4757.14;  // 高速射球目標轉速（每分鐘轉數）
    private static final double VELOCITY_TOLERANCE = 100;  // 速度誤差容許範圍（ticks/sec）
    private static final double FEEDER_POWER = 1.0;  // 送球馬達功率
    private static final double INTAKE_POWER = 1.0;  // 吸球馬達功率

    // ===== 狀態變數 =====
    private double targetRPM = 0;  // 目標轉速
    private boolean shooterOn = false;  // 射球馬達是否啟動
    private boolean feedEnabled = false;  // 送球機構是否啟動

    // ===== 按鍵邊緣檢測 =====
    private boolean prevX = false;  // 上一幀X鍵狀態（用於偵測按下瞬間）
    private boolean prevRightBumper = false;  // 上一幀RB鍵狀態

    @Override
    public void runOpMode() {
        initializeHardware();  // 初始化所有硬體
        displayInitMessage();  // 顯示初始化訊息
        waitForStart();  // 等待比賽開始

        // 主控制迴圈
        while (opModeIsActive()) {
            handleDriveControls();  // 處理底盤控制
            handleShooterControls();  // 處理射球控制
            handleFeederControls();  // 處理送球控制
            handleIntakeControls();  // 處理吸球控制
            updateTelemetry();  // 更新遙測數據
        }

        stopAllMotors();  // 停止所有馬達
    }

    /**
     * 初始化所有硬體設備
     */
    private void initializeHardware() {
        initDriveMotors();  // 初始化底盤馬達
        initShooterMotor();  // 初始化射球馬達
        initIntakeMotors();  // 初始化吸球馬達
    }

    /**
     * 初始化底盤四個驅動馬達
     */
    private void initDriveMotors() {
        // 從硬體映射取得馬達
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // 設定左側馬達反轉（因為安裝方向相反）
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // 統一設定所有底盤馬達
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // 停止時煞車
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // 不使用編碼器模式
        }
    }

    /**
     * 初始化射球馬達
     */
    private void initShooterMotor() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // 重置編碼器
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);  // 使用編碼器控制速度
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // 停止時自由滑行
    }

    /**
     * 初始化吸球與送球馬達
     */
    private void initIntakeMotors() {
        intake1 = hardwareMap.get(DcMotor.class, "intake1");  // 吸球馬達
        intake2 = hardwareMap.get(DcMotor.class, "intake2");  // 送球馬達

        // 設定馬達方向
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        // 統一設定intake馬達
        for (DcMotor motor : new DcMotor[]{intake1, intake2}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // 停止時自由滑行
        }
    }

    /**
     * 顯示初始化完成訊息與操作說明
     */
    private void displayInitMessage() {
        telemetry.addData("狀態", "✓ 初始化完成");
        telemetry.addLine("【射球機構】");
        telemetry.addLine("  X: 啟動 | Y(按住): 送球 | RB: 停止");
        telemetry.addLine("【吸球機構】");
        telemetry.addLine("  A/LB: 吸球 | B: 吐球");
        telemetry.update();
    }

    /**
     * 處理底盤麥克納姆輪控制
     * 左搖桿控制前後左右移動，右搖桿控制旋轉
     */
    private void handleDriveControls() {
        // 讀取手把輸入（Y軸需反轉）
        double forward = -gamepad1.left_stick_y;  // 前後移動
        double strafe = gamepad1.left_stick_x;    // 左右平移
        double rotate = gamepad1.right_stick_x;   // 旋轉

        // 麥克納姆輪運動學計算
        double[] powers = {
                forward + rotate + strafe,  // FL 前左輪
                forward - rotate - strafe,  // FR 前右輪
                forward + rotate - strafe,  // BL 後左輪
                forward - rotate + strafe   // BR 後右輪
        };

        // 正規化功率值（確保不超過1.0）
        double max = 1.0;
        for (double p : powers) max = Math.max(max, Math.abs(p));

        // 設定馬達功率
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setPower(powers[i] / max);
        }
    }

    /**
     * 處理射球馬達控制
     * X鍵啟動，RB鍵緊急停止
     */
    private void handleShooterControls() {
        // 邊緣觸發：只在按下瞬間執行
        if (gamepad1.x && !prevX) activateShooter();  // X鍵啟動射球
        if (gamepad1.right_bumper && !prevRightBumper) emergencyStop();  // RB緊急停止

        // 更新按鍵狀態
        prevX = gamepad1.x;
        prevRightBumper = gamepad1.right_bumper;

        // 設定馬達速度（開啟時使用目標速度，否則為0）
        shooterMotor.setVelocity(shooterOn ? calculateTargetVelocity(targetRPM) : 0);
    }

    /**
     * 啟動射球馬達
     */
    private void activateShooter() {
        shooterOn = true;
        targetRPM = HIGH_RPM;  // 設定為高速模式
    }

    /**
     * 緊急停止所有機構
     */
    private void emergencyStop() {
        shooterOn = false;
        feedEnabled = false;
        targetRPM = 0;
        shooterMotor.setVelocity(0);
        intake1.setPower(0);
        intake2.setPower(0);
    }

    /**
     * 處理送球機構控制
     * 只有在射球馬達達到目標速度時才能送球
     */
    private void handleFeederControls() {
        // 送球條件：Y鍵按住 && 射球馬達啟動 && 速度達標
        boolean canFeed = gamepad1.y && shooterOn && isVelocityInRange();
        feedEnabled = canFeed;
        intake2.setPower(canFeed ? FEEDER_POWER : 0);
    }

    /**
     * 檢查射球馬達速度是否在容許範圍內
     */
    private boolean isVelocityInRange() {
        double error = Math.abs(shooterMotor.getVelocity() - calculateTargetVelocity(targetRPM));
        return error <= VELOCITY_TOLERANCE;
    }

    /**
     * 處理吸球馬達控制
     * A或LB鍵吸球，B鍵吐球
     */
    private void handleIntakeControls() {
        double power = 0;
        if (gamepad1.a || gamepad1.left_bumper) power = INTAKE_POWER;  // 吸球
        else if (gamepad1.b) power = -INTAKE_POWER;  // 吐球
        intake1.setPower(power);
    }

    /**
     * 計算當前射球馬達轉速（RPM）
     */
    private double calculateRPM() {
        return (shooterMotor.getVelocity() / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    /**
     * 將RPM轉換為馬達速度（ticks/sec）
     */
    private double calculateTargetVelocity(double rpm) {
        return rpm * (SHOOTER_TICKS_PER_REV / 60.0);
    }

    /**
     * 更新遙測數據到Driver Station
     */
    private void updateTelemetry() {
        double rpm = calculateRPM();
        double error = rpm - targetRPM;

        telemetry.addData("Shooter", shooterOn ? "🟢 ON" : "🔴 OFF");
        telemetry.addData("送球", feedEnabled ? "✓ 進行中" : "✗ 待命");
        telemetry.addData("RPM", "%.0f / %.0f", rpm, targetRPM);
        telemetry.addData("狀態", isVelocityInRange() ? "✓ 達標" : "✗ 未達標 (%.0f)", error);
        telemetry.update();
    }

    /**
     * 停止所有馬達（程式結束時呼叫）
     */
    private void stopAllMotors() {
        for (DcMotor motor : driveMotors) motor.setPower(0);
        shooterMotor.setVelocity(0);
        intake1.setPower(0);
        intake2.setPower(0);
    }
}
