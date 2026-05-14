package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test_Arm", group = "Test")
public class Test_Arm extends LinearOpMode {
    private Servo ArmLeft, ArmRight;
    private Servo OutputClaw;

    private static final double SERVO_MIN = 0.05;
    private static final double SERVO_MAX = 0.95;
    private static final double START_ARM_ANGLE = 0.20;
    private static final double START_WRIST_OFFSET = 0.00;
    private static final double STICK_STEP = 0.006;
    private static final double FINE_STEP = 0.001;

    private static final double CLAW_OPEN_POSITION = 0.85;
    private static final double CLAW_CLOSE_POSITION = 0.68;

    private double armAngle = START_ARM_ANGLE;
    private double wristOffset = START_WRIST_OFFSET;
    private double leftTarget = START_ARM_ANGLE;
    private double rightTarget = START_ARM_ANGLE;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            armAngle += -gamepad2.left_stick_y * STICK_STEP;
            wristOffset += gamepad2.right_stick_y * STICK_STEP;

            if (gamepad2.dpad_up) {
                armAngle += FINE_STEP;
            }
            if (gamepad2.dpad_down) {
                armAngle -= FINE_STEP;
            }
            if (gamepad2.dpad_right) {
                wristOffset += FINE_STEP;
            }
            if (gamepad2.dpad_left) {
                wristOffset -= FINE_STEP;
            }

            if (gamepad2.left_bumper) {
                armAngle = START_ARM_ANGLE;
                wristOffset = START_WRIST_OFFSET;
            }

            armAngle = clamp(armAngle, SERVO_MIN, SERVO_MAX);
            leftTarget = clamp(armAngle + wristOffset, SERVO_MIN, SERVO_MAX);
            rightTarget = clamp(armAngle - wristOffset, SERVO_MIN, SERVO_MAX);

            ArmLeft.setPosition(leftTarget);
            ArmRight.setPosition(rightTarget);

            if (gamepad2.a) {
                OutputClaw.setPosition(CLAW_OPEN_POSITION);
            }
            if (gamepad2.b) {
                OutputClaw.setPosition(CLAW_CLOSE_POSITION);
            }

            telemetry.addLine("Differential Arm Tuning");
            telemetry.addLine("G2 left stick Y: armAngle average");
            telemetry.addLine("G2 right stick Y: wristOffset difference");
            telemetry.addLine("G2 dpad: fine tune arm/wrist");
            telemetry.addLine("G2 left bumper: reset arm/wrist");
            telemetry.addLine("G2 A / B: output claw open / close");
            telemetry.addData("armAngle", "%.3f", armAngle);
            telemetry.addData("wristOffset", "%.3f", wristOffset);
            telemetry.addData("ArmLeft target", "%.3f", leftTarget);
            telemetry.addData("ArmRight target", "%.3f", rightTarget);
            telemetry.addData("ArmLeft actual", "%.3f", ArmLeft.getPosition());
            telemetry.addData("ArmRight actual", "%.3f", ArmRight.getPosition());
            telemetry.addData("OutputClawPosition", "%.3f", OutputClaw.getPosition());
            telemetry.addData("Servo safe range", "%.2f ~ %.2f", SERVO_MIN, SERVO_MAX);
            telemetry.update();

            idle();
        }
    }

    private void initHardware() {
        ArmLeft = hardwareMap.get(Servo.class, "ArmLeft");
        ArmRight = hardwareMap.get(Servo.class, "ArmRight");
        ArmRight.setDirection(Servo.Direction.REVERSE);

        OutputClaw = hardwareMap.get(Servo.class, "OutputClaw");

        ArmLeft.setPosition(leftTarget);
        ArmRight.setPosition(rightTarget);
        OutputClaw.setPosition(CLAW_OPEN_POSITION);
    }

    private double clamp(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }
}
