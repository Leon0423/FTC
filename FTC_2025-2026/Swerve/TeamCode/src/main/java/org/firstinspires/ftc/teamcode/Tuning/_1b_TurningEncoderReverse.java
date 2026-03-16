package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/**
 * This is a simple teleop program for testing the turning motors of a swerve drive.
 * Pressing the 'A' button on gamepad1 will apply a small positive power to all turning motors.
 * Telemetry will display the current encoder position of each turning motor.
 *
 * This is useful for verifying the motor direction and encoder counting direction.
 * If a motor turns forward but its encoder count goes down, you need to reverse the encoder direction
 * in Constants.java (e.g., kFrontLeftTurningEncoderReversed = true).
 */
@TeleOp(name = "1b. Turning Encoder Reverse", group = "Tuning")
public class _1b_TurningEncoderReverse extends LinearOpMode {

    private CRServo frontLeftTurn, frontRightTurn, backLeftTurn, backRightTurn;
    private AnalogInput flAbs, frAbs, blAbs, brAbs;

    private final ElapsedTime pulseTimer = new ElapsedTime();
    private boolean pulseActive = false;
    private boolean lastAPressed = false;
    private boolean lastRb = false;
    private boolean lastLb = false;
    private double pulsePower = 0.3; // adjustable in 10% steps

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the turning motors from the hardware map
        frontLeftTurn = hardwareMap.get(CRServo.class, DriveConstants.kFrontLeftTurningMotorName);
        frontRightTurn = hardwareMap.get(CRServo.class, DriveConstants.kFrontRightTurningMotorName);
        backLeftTurn = hardwareMap.get(CRServo.class, DriveConstants.kBackLeftTurningMotorName);
        backRightTurn = hardwareMap.get(CRServo.class, DriveConstants.kBackRightTurningMotorName);

        // Absolute encoders (Axon feedback to analog ports)
        flAbs = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontLeftAbsoluteEncoderName);
        frAbs = hardwareMap.get(AnalogInput.class, DriveConstants.kFrontRightAbsoluteEncoderName);
        blAbs = hardwareMap.get(AnalogInput.class, DriveConstants.kBackLeftAbsoluteEncoderName);
        brAbs = hardwareMap.get(AnalogInput.class, DriveConstants.kBackRightAbsoluteEncoderName);

        // Set motor directions (this is for motor rotation, not encoder counting)
        // We assume a standard setup here. If motors turn the wrong way, you might adjust here or in your module logic.
        frontLeftTurn.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightTurn.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftTurn.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightTurn.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Press 'A' to pulse +power (uses adjustable value).");
        telemetry.addLine("LB/RB: adjust power in ±10% steps (clamped to [-1,1]).");
        telemetry.addLine("Watch voltage/angle to decide encoder reverse and offsets.");
        telemetry.update();

        waitForStart();
        pulseTimer.reset();

        while (opModeIsActive()) {
            // Edge-detect bumper presses to adjust pulse power in 10% steps
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            if (rb && !lastRb) {
                pulsePower = Math.min(1.0, pulsePower + 0.1);
            }
            if (lb && !lastLb) {
                pulsePower = Math.max(-1.0, pulsePower - 0.1);
            }
            lastRb = rb;
            lastLb = lb;

            boolean aPressed = gamepad1.a;

            // Rising edge on A starts a short pulse at current pulsePower (sign respected)
            if (aPressed && !lastAPressed) {
                pulseActive = true;
                pulseTimer.reset();
            }
            lastAPressed = aPressed;

            double power = 0;
            if (pulseActive) {
                power = pulsePower;
                if (pulseTimer.milliseconds() > 300) { // ~0.3s pulse duration
                    pulseActive = false;
                    power = 0;
                }
            }

            frontLeftTurn.setPower(power);
            frontRightTurn.setPower(power);
            backLeftTurn.setPower(power);
            backRightTurn.setPower(power);

            telemetry.addData("Pulse Power (set)", "%.1f%%", pulsePower * 100);
            telemetry.addData("Active Power (output)", "%.2f", power);
            telemetry.addLine("FL rawV/deg | normDeg");
            telemetry.addData("FL", "%.3fV | %.1f | %.1f", flAbs.getVoltage(), rawDeg(flAbs), normDeg(flAbs, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed));
            telemetry.addLine("FR rawV/deg | normDeg");
            telemetry.addData("FR", "%.3fV | %.1f | %.1f", frAbs.getVoltage(), rawDeg(frAbs), normDeg(frAbs, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, DriveConstants.kFrontRightDriveAbsoluteEncoderReversed));
            telemetry.addLine("BL rawV/deg | normDeg");
            telemetry.addData("BL", "%.3fV | %.1f | %.1f", blAbs.getVoltage(), rawDeg(blAbs), normDeg(blAbs, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, DriveConstants.kBackLeftDriveAbsoluteEncoderReversed));
            telemetry.addLine("BR rawV/deg | normDeg");
            telemetry.addData("BR", "%.3fV | %.1f | %.1f", brAbs.getVoltage(), rawDeg(brAbs), normDeg(brAbs, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, DriveConstants.kBackRightDriveAbsoluteEncoderReversed));
            telemetry.update();
        }

        // Stop motors on exit
        frontLeftTurn.setPower(0);
        frontRightTurn.setPower(0);
        backLeftTurn.setPower(0);
        backRightTurn.setPower(0);
    }

    private double rawDeg(AnalogInput input) {
        double angle = (input.getVoltage() / input.getMaxVoltage()) * 360.0;
        return angle;
    }

    private double normDeg(AnalogInput input, double offsetRad, boolean reversed) {
        double angle = (input.getVoltage() / input.getMaxVoltage()) * 2.0 * Math.PI;
        angle -= offsetRad;
        if (reversed) angle = -angle;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return Math.toDegrees(angle);
    }
}
