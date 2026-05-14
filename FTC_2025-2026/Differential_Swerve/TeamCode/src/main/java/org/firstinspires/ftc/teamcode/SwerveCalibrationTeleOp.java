package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Constants.DIFFERENTIAL_DRIVE_POWER_SCALE;
import static org.firstinspires.ftc.teamcode.Constants.DIFFERENTIAL_STEER_POWER_SCALE;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Constants.LEFT_MOTOR_A;
import static org.firstinspires.ftc.teamcode.Constants.LEFT_MOTOR_A_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.LEFT_MOTOR_B;
import static org.firstinspires.ftc.teamcode.Constants.LEFT_MOTOR_B_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.MAX_DRIVE_SPEED_MPS;
import static org.firstinspires.ftc.teamcode.Constants.MAX_ROTATION_SPEED_RPS;
import static org.firstinspires.ftc.teamcode.Constants.RIGHT_MOTOR_A;
import static org.firstinspires.ftc.teamcode.Constants.RIGHT_MOTOR_A_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.RIGHT_MOTOR_B;
import static org.firstinspires.ftc.teamcode.Constants.RIGHT_MOTOR_B_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.STEER_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Constants.STEER_KD;
import static org.firstinspires.ftc.teamcode.Constants.STEER_KI;
import static org.firstinspires.ftc.teamcode.Constants.STEER_KP;
import static org.firstinspires.ftc.teamcode.Constants.STEER_TOLERANCE_RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.TICKS_PER_MOTOR_REV;
import static org.firstinspires.ftc.teamcode.Constants.TICKS_TO_METERS_DRIVE;
import static org.firstinspires.ftc.teamcode.Constants.TICKS_TO_RADIANS_STEER;
import static org.firstinspires.ftc.teamcode.Constants.TRACK_WIDTH_METERS;
import static org.firstinspires.ftc.teamcode.Constants.WHEEL_DIAMETER_METERS;

@TeleOp(name = "Swerve Calibration", group = "Test")
public class SwerveCalibrationTeleOp extends OpMode {
    private static final double POWER_SCALE = 0.35;
    private static final double DEADBAND = 0.05;

    private Module leftModule;
    private Module rightModule;
    private Target target = Target.BOTH;
    private boolean previousY;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;

    @Override
    public void init() {
        leftModule = new Module(
                LEFT_MOTOR_A,
                LEFT_MOTOR_B,
                LEFT_MOTOR_A_INVERTED,
                LEFT_MOTOR_B_INVERTED
        );
        rightModule = new Module(
                RIGHT_MOTOR_A,
                RIGHT_MOTOR_B,
                RIGHT_MOTOR_A_INVERTED,
                RIGHT_MOTOR_B_INVERTED
        );

        leftModule.init();
        rightModule.init();

        telemetry.addLine("Swerve calibration ready");
        telemetry.addData("Controls", "LY drive, RX steer, dpad left/right target, Y reset");
        telemetry.update();
    }

    @Override
    public void loop() {
        updateTargetSelection();

        if (gamepad1.y && !previousY) {
            leftModule.resetEncoders();
            rightModule.resetEncoders();
        }
        previousY = gamepad1.y;

        double drive = applyDeadband(-gamepad1.left_stick_y) * POWER_SCALE;
        double steer = applyDeadband(gamepad1.right_stick_x) * POWER_SCALE;

        if (target.includesLeft()) {
            leftModule.applyDifferential(drive, steer);
        } else {
            leftModule.stop();
        }

        if (target.includesRight()) {
            rightModule.applyDifferential(drive, steer);
        } else {
            rightModule.stop();
        }

        sendTelemetry(drive, steer);
    }

    @Override
    public void stop() {
        leftModule.stop();
        rightModule.stop();
    }

    private void updateTargetSelection() {
        if (gamepad1.dpad_left && !previousDpadLeft) {
            target = target.previous();
        }
        if (gamepad1.dpad_right && !previousDpadRight) {
            target = target.next();
        }

        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;
    }

    private void sendTelemetry(double drive, double steer) {
        telemetry.addData("Target", target.label);
        telemetry.addData("Controls", "LY drive, RX steer, dpad target, Y reset encoders");
        telemetry.addData("command drive", "%.3f", drive);
        telemetry.addData("command steer", "%.3f", steer);

        telemetry.addLine();
        telemetry.addLine("Constants");
        telemetry.addData("track width m", "%.4f", TRACK_WIDTH_METERS);
        telemetry.addData("wheel diameter m", "%.4f", WHEEL_DIAMETER_METERS);
        telemetry.addData("ticks per motor rev", "%.1f", TICKS_PER_MOTOR_REV);
        telemetry.addData("steer gear ratio", "%.4f", STEER_GEAR_RATIO);
        telemetry.addData("drive gear ratio", "%.4f", DRIVE_GEAR_RATIO);
        telemetry.addData("steer power scale", "%.4f", DIFFERENTIAL_STEER_POWER_SCALE);
        telemetry.addData("drive power scale", "%.4f", DIFFERENTIAL_DRIVE_POWER_SCALE);
        telemetry.addData("ticks to steer rad", "%.8f", TICKS_TO_RADIANS_STEER);
        telemetry.addData("ticks to drive m", "%.8f", TICKS_TO_METERS_DRIVE);
        telemetry.addData("max drive m/s", "%.3f", MAX_DRIVE_SPEED_MPS);
        telemetry.addData("max rotation rad/s", "%.3f", MAX_ROTATION_SPEED_RPS);
        telemetry.addData("steer PID", "P %.4f  I %.4f  D %.4f", STEER_KP, STEER_KI, STEER_KD);
        telemetry.addData("steer tolerance deg", "%.2f", Math.toDegrees(STEER_TOLERANCE_RADIANS));

        telemetry.addLine();
        telemetry.addLine("Left Module");
        leftModule.addTelemetry("left");

        telemetry.addLine();
        telemetry.addLine("Right Module");
        rightModule.addTelemetry("right");

        telemetry.update();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }

    private enum Target {
        LEFT("left"),
        RIGHT("right"),
        BOTH("both");

        private final String label;

        Target(String label) {
            this.label = label;
        }

        private boolean includesLeft() {
            return this == LEFT || this == BOTH;
        }

        private boolean includesRight() {
            return this == RIGHT || this == BOTH;
        }

        private Target next() {
            return values()[(ordinal() + 1) % values().length];
        }

        private Target previous() {
            return values()[(ordinal() + values().length - 1) % values().length];
        }
    }

    private class Module {
        private final String motorAName;
        private final String motorBName;
        private final boolean motorAInverted;
        private final boolean motorBInverted;
        private DcMotorEx motorA;
        private DcMotorEx motorB;

        private Module(String motorAName, String motorBName, boolean motorAInverted, boolean motorBInverted) {
            this.motorAName = motorAName;
            this.motorBName = motorBName;
            this.motorAInverted = motorAInverted;
            this.motorBInverted = motorBInverted;
        }

        private void init() {
            motorA = hardwareMap.get(DcMotorEx.class, motorAName);
            motorB = hardwareMap.get(DcMotorEx.class, motorBName);

            motorA.setDirection(motorAInverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motorB.setDirection(motorBInverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            resetEncoders();
        }

        private void resetEncoders() {
            motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        private void applyDifferential(double drive, double steer) {
            double scaledDrive = drive * DIFFERENTIAL_DRIVE_POWER_SCALE;
            double scaledSteer = steer * DIFFERENTIAL_STEER_POWER_SCALE;

            double powerA = scaledSteer + scaledDrive;
            double powerB = scaledSteer - scaledDrive;

            double maxPower = Math.max(Math.abs(powerA), Math.abs(powerB));
            if (maxPower > 1.0) {
                powerA /= maxPower;
                powerB /= maxPower;
            }

            motorA.setPower(powerA);
            motorB.setPower(powerB);
        }

        private void stop() {
            motorA.setPower(0.0);
            motorB.setPower(0.0);
        }

        private double getSteerTicks() {
            return (motorA.getCurrentPosition() + motorB.getCurrentPosition()) / 2.0;
        }

        private double getDriveTicks() {
            return (motorA.getCurrentPosition() - motorB.getCurrentPosition()) / 2.0;
        }

        private double getAngleRadians() {
            return getSteerTicks() * TICKS_TO_RADIANS_STEER;
        }

        private double getDriveMeters() {
            return getDriveTicks() * TICKS_TO_METERS_DRIVE;
        }

        private double getDriveMetersPerSecond() {
            return ((motorA.getVelocity() - motorB.getVelocity()) / 2.0) * TICKS_TO_METERS_DRIVE;
        }

        private void addTelemetry(String prefix) {
            telemetry.addData(prefix + " motor A", "%s inv=%s ticks=%d vel=%.1f",
                    motorAName,
                    motorAInverted,
                    motorA.getCurrentPosition(),
                    motorA.getVelocity());
            telemetry.addData(prefix + " motor B", "%s inv=%s ticks=%d vel=%.1f",
                    motorBName,
                    motorBInverted,
                    motorB.getCurrentPosition(),
                    motorB.getVelocity());
            telemetry.addData(prefix + " steer ticks", "%.1f", getSteerTicks());
            telemetry.addData(prefix + " drive ticks", "%.1f", getDriveTicks());
            telemetry.addData(prefix + " angle deg", "%.2f", Math.toDegrees(getAngleRadians()));
            telemetry.addData(prefix + " angle rad", "%.4f", getAngleRadians());
            telemetry.addData(prefix + " drive meters", "%.4f", getDriveMeters());
            telemetry.addData(prefix + " drive m/s", "%.4f", getDriveMetersPerSecond());
        }
    }
}
