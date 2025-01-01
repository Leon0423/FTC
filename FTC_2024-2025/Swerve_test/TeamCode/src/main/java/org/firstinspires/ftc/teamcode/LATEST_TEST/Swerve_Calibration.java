package org.firstinspires.ftc.teamcode.LATEST_TEST;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Swerve Calibration", group = "Calibration")
public class Swerve_Calibration extends OpMode {
    // Servos
    private Servo frontLeftServo;
    private Servo frontRightServo;
    private Servo backLeftServo;
    private Servo backRightServo;

    // 當前選擇的servo
    private int currentServoIndex = 0;
    private Servo currentServo;

    // Offset值
    private double frontLeftOffset = 0.0;
    private double frontRightOffset = 0.0;
    private double backLeftOffset = 0.0;
    private double backRightOffset = 0.0;

    // 控制常數
    private static final double SERVO_ADJUST_SPEED = 0.5; // 每秒轉動度數
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;
    private static final double SERVO_RANGE_DEGREES = 300.0; // AXON MINI+ 的範圍

    private String[] servoNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
    private long lastUpdateTime;

    @Override
    public void init() {
        // 初始化servos
        frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(Servo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(Servo.class, "backLeftServo");
        backRightServo = hardwareMap.get(Servo.class, "backRightServo");

        currentServo = frontLeftServo;
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // 轉換為秒
        lastUpdateTime = currentTime;

        // 選擇Servo
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            currentServoIndex = (currentServoIndex - 1 + 4) % 4;
            sleep(200); // 防彈跳
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            currentServoIndex = (currentServoIndex + 1) % 4;
            sleep(200); // 防彈跳
        }

        // 更新當前選擇的Servo
        switch (currentServoIndex) {
            case 0:
                currentServo = frontLeftServo;
                break;
            case 1:
                currentServo = frontRightServo;
                break;
            case 2:
                currentServo = backLeftServo;
                break;
            case 3:
                currentServo = backRightServo;
                break;
        }

        // 調整Offset
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            adjustOffset(deltaTime, 1);
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            adjustOffset(deltaTime, -1);
        }

        // 微調當前位置
        if (gamepad1.right_trigger > 0.1) {
            adjustPosition(deltaTime, gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            adjustPosition(deltaTime, -gamepad1.left_trigger);
        }

        // 重置當前Servo到0度位置
        if (gamepad1.a) {
            currentServo.setPosition(0.5); // 中間位置
            sleep(200);
        }

        // 顯示資訊
        telemetry.addData("Selected Servo", servoNames[currentServoIndex]);
        telemetry.addData("Current Position", "%.3f", currentServo.getPosition());
        telemetry.addData("Offsets", "FL: %.1f, FR: %.1f, BL: %.1f, BR: %.1f",
                frontLeftOffset, frontRightOffset, backLeftOffset, backRightOffset);

        telemetry.addLine("\nControls:");
        telemetry.addLine("DPAD Up/Down: Select Servo");
        telemetry.addLine("Bumpers: Adjust Offset");
        telemetry.addLine("Triggers: Fine-tune Position");
        telemetry.addLine("A: Reset to Center");

        telemetry.update();
    }

    private void adjustOffset(double deltaTime, int direction) {
        double adjustment = SERVO_ADJUST_SPEED * deltaTime * direction;

        switch (currentServoIndex) {
            case 0:
                frontLeftOffset = adjustOffsetValue(frontLeftOffset, adjustment);
                break;
            case 1:
                frontRightOffset = adjustOffsetValue(frontRightOffset, adjustment);
                break;
            case 2:
                backLeftOffset = adjustOffsetValue(backLeftOffset, adjustment);
                break;
            case 3:
                backRightOffset = adjustOffsetValue(backRightOffset, adjustment);
                break;
        }
    }

    private double adjustOffsetValue(double currentOffset, double adjustment) {
        double newOffset = currentOffset + adjustment;
        // 保持在-180到180度範圍內
        if (newOffset > 180) newOffset -= 360;
        if (newOffset < -180) newOffset += 360;
        return newOffset;
    }

    private void adjustPosition(double deltaTime, double amount) {
        double currentPos = currentServo.getPosition();
        double newPos = currentPos + (amount * SERVO_ADJUST_SPEED * deltaTime);
        currentServo.setPosition(Range.clip(newPos, SERVO_MIN_POS, SERVO_MAX_POS));
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            // 忽略中斷異常
        }
    }
}