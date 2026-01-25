package org.firstinspires.ftc.teamcode.flyWheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Velocity Test", group = "Test")
public class FlywheelVelocityTest extends LinearOpMode {

    private DcMotorEx flywheelMotor;

    // 當前測試速度
    private double testVelocity = 0;

    // 速度調整步長
    private double velocityStep = 50;  // 每次增減50 ticks/s

    private static final double COUNTS_PER_REV = 28;

    @Override
    public void runOpMode() throws InterruptedException {

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        // 重要：使用 RUN_USING_ENCODER 會自動用內建PIDF
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("控制說明:");
        telemetry.addLine("D-Pad 左/右: 微調速度 (±50)");
        telemetry.addLine("D-Pad 上/下: 大幅調整 (±200)");
        telemetry.addLine();
        telemetry.addLine("RB: 停止飛輪 (0)");
        telemetry.addLine("X: 低速測試 (1000)");
        telemetry.addLine("Y: 高速測試 (2000)");
        telemetry.addLine();
        telemetry.addLine("A: 正轉");
        telemetry.addLine("B: 反轉");
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
            if (gamepad1.right_bumper) {
                testVelocity = 0;
            }
            if (gamepad1.x) {
                testVelocity = 1000;
            }
            if (gamepad1.y) {
                testVelocity = 2000;
            }

            if (gamepad1.aWasPressed()) {
                flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (gamepad1.bWasPressed()){
                flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            // === 設定馬達速度 ===
            flywheelMotor.setVelocity(testVelocity);

            // === 讀取實際速度 ===
            double currentVelocity = flywheelMotor.getVelocity();
            double currentRPM = (currentVelocity / COUNTS_PER_REV) * 60;
            double targetRPM = (testVelocity / COUNTS_PER_REV) * 60;

            // === 顯示資訊 ===
            telemetry.addData("設定速度", "%.0f ticks/s", testVelocity);
            telemetry.addData("實際速度", "%.0f ticks/s", currentVelocity);
            telemetry.addLine("========================");
            telemetry.addData("設定RPM", "%.0f RPM", targetRPM);
            telemetry.addData("實際RPM", "%.0f RPM", currentRPM);
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("========================");
            telemetry.addData("✏ 記錄此速度&RPM", "%.0f ticks/s (%.0f RPM)", testVelocity, targetRPM);

            telemetry.update();
        }

        // 程式結束時停止馬達
        flywheelMotor.setVelocity(0);
    }
}