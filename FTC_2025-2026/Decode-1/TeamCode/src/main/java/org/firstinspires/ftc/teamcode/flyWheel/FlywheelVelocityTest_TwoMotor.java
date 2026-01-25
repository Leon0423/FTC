package org.firstinspires.ftc.teamcode.flyWheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Velocity Test TwoMotor", group = "Test")
public class FlywheelVelocityTest_TwoMotor extends LinearOpMode {

    private DcMotorEx flywheelMotorLeft, flywheelMotorRight;

    // 當前測試 RPM
    private double testRPM = 0;

    // RPM 調整步長
    private double rpmSmallStep = 100;  // 每次增減 100 RPM
    private double rpmLargeStep = 500;  // 每次增減 500 RPM

    // 計算
    private static final double COUNTS_PER_REV = 28;

    @Override
    public void runOpMode() throws InterruptedException {

        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "SL");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "SB");

        // 重要：使用 RUN_USING_ENCODER 會自動用內建PIDF
        flywheelMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("控制說明:");
        telemetry.addLine("D-Pad 左/右: 微調RPM (±100)");
        telemetry.addLine("D-Pad 上/下: 大幅調整 (±500)");
        telemetry.addLine();
        telemetry.addLine("Right_Bumper: 停止飛輪 (0)");
        telemetry.addLine("X: 低速測試 (2000 RPM)");
        telemetry.addLine("Y: 高速測試 (4000 RPM)");
        telemetry.addLine();
        telemetry.addLine("A: 正轉");
        telemetry.addLine("B: 反轉");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // D-Pad 左右：微調 RPM
            if (gamepad1.dpadLeftWasPressed()){
                testRPM -= rpmSmallStep;
            }
            if (gamepad1.dpadRightWasPressed()){
                testRPM += rpmSmallStep;
            }

            // D-Pad 上下：大幅調整
            if (gamepad1.dpadUpWasPressed()) {
                testRPM += rpmLargeStep;
            }
            if (gamepad1.dpadDownWasPressed()) {
                testRPM -= rpmLargeStep;
            }

            // 快速設定按鈕
            if (gamepad1.right_bumper) {
                testRPM = 0;
            }
            if (gamepad1.x) {
                testRPM = 2000;
            }
            if (gamepad1.y) {
                testRPM = 4000;
            }

            if (gamepad1.aWasPressed()) {
                flywheelMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                flywheelMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (gamepad1.bWasPressed()){
                flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                flywheelMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            // === 將 RPM 轉換為 ticks/s 並設定馬達速度 ===
            double targetVelocity = (testRPM / 60) * COUNTS_PER_REV;
            flywheelMotorLeft.setVelocity(targetVelocity);
            flywheelMotorRight.setVelocity(targetVelocity);

            // === 讀取實際速度 ===
            double currentVelocity_Left = flywheelMotorLeft.getVelocity();
            double currentVelocity_Right = flywheelMotorRight.getVelocity();
            double currentRPM_Left = (currentVelocity_Left / COUNTS_PER_REV) * 60;
            double currentRPM_Right = (currentVelocity_Right / COUNTS_PER_REV) * 60;

            // === 顯示資訊 ===
            telemetry.addData("目標RPM", "%.0f RPM", testRPM);
            telemetry.addLine("========================");
            telemetry.addLine("實際RPM");
            telemetry.addData("currentRPM_Left", "%.0f RPM", currentRPM_Left);
            telemetry.addData("currentRPM_Right", "%.0f RPM", currentRPM_Right);
            telemetry.addLine();
            telemetry.addLine("實際速度");
            telemetry.addData("目標Velocity", "%.0f ticks/s", targetVelocity);
            telemetry.addData("Velocity_Left", "%.0f ticks/s", currentVelocity_Left);
            telemetry.addData("Velocity_Right", "%.0f ticks/s", currentVelocity_Right);
            telemetry.addLine();
            telemetry.update();
        }

        // 程式結束時停止馬達
        flywheelMotorLeft.setVelocity(0);
        flywheelMotorRight.setVelocity(0);
    }
}
