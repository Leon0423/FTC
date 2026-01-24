package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Velocity Test TwoMotor", group = "Test")
public class FlywheelVelocityTest_twomotor extends LinearOpMode {

    private DcMotorEx flywheelMotorLeft, flywheelMotorRight;

    // 當前測試速度
    private double testVelocity = 0;

    // 速度調整步長
    private double velocityStep = 50;  // 每次增減50 ticks/s

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
        telemetry.addLine("D-Pad 左/右: 微調速度 (±50)");
        telemetry.addLine("D-Pad 上/下: 大幅調整 (±200)");
        telemetry.addLine();
        telemetry.addLine("A: 停止飛輪 (0)");
        telemetry.addLine("X: 低速測試 (1000)");
        telemetry.addLine("Y: 高速測試 (2000)");
        telemetry.addLine();
        telemetry.addLine("按 START 開始測試");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 右搖桿：微調速度
            if (gamepad1.dpadLeftWasPressed()){
                testVelocity -= 50;
            }
            if (gamepad1.dpadRightWasPressed()){
                testVelocity += 50;
            }

            // D-Pad：大幅調整
            if (gamepad1.dpadUpWasPressed()) {
                testVelocity += 200;
            }
            if (gamepad1.dpadDownWasPressed()) {
                testVelocity -= 200;
            }

            // 快速設定按鈕
            if (gamepad1.a) {
                testVelocity = 0;
            }
            if (gamepad1.x) {
                testVelocity = 1000;
            }
            if (gamepad1.y) {
                testVelocity = 2000;
            }

            // === 設定馬達速度 ===
            flywheelMotorLeft.setVelocity(testVelocity);
            flywheelMotorRight.setVelocity(testVelocity);

            // === 讀取實際速度 ===
            double currentVelocity_Left = flywheelMotorLeft.getVelocity();
            double currentVelocity_Right = flywheelMotorRight.getVelocity();
            double currentRPM_Left = (currentVelocity_Left / COUNTS_PER_REV) * 60;
            double currentRPM_Right = (currentVelocity_Right / COUNTS_PER_REV) * 60;
            double targetRPM = (testVelocity / COUNTS_PER_REV) * 60;

            // === 顯示資訊 ===
            telemetry.addData("目標速度", "%.0f ticks/s", testVelocity);
            telemetry.addLine("實際速度");
            telemetry.addData("Velocity_Left", "%.0f ticks/s", currentVelocity_Left);
            telemetry.addData("Velocity_Right", "%.0f ticks/s", currentVelocity_Right);
            telemetry.addLine("========================");
            telemetry.addData("目標RPM", "%.0f RPM", targetRPM);
            telemetry.addLine("實際RPM");
            telemetry.addData("currentRPM_Left", "%.0f RPM", currentRPM_Left);
            telemetry.addData("currentRPM_Right", "%.0f RPM", currentRPM_Right);
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("========================");
            telemetry.addData("✏ 記錄此速度&RPM", "%.0f ticks/s (%.0f RPM)", testVelocity, targetRPM);
            telemetry.update();
        }

        // 程式結束時停止馬達
        flywheelMotorLeft.setVelocity(0);
        flywheelMotorRight.setVelocity(0);
    }
}
