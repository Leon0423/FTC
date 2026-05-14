package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveModuleState;

import static org.firstinspires.ftc.teamcode.Constants.DIFFERENTIAL_DRIVE_POWER_SCALE;
import static org.firstinspires.ftc.teamcode.Constants.DIFFERENTIAL_STEER_POWER_SCALE;
import static org.firstinspires.ftc.teamcode.Constants.MAX_DRIVE_SPEED_MPS;
import static org.firstinspires.ftc.teamcode.Constants.MODULE_OPTIMIZE_MIN_SPEED_MPS;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_CLOSED_LOOP_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_MANUAL_POWER_SCALE;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_MOTOR_A;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_MOTOR_A_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_MOTOR_B;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_MOTOR_B_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_RIGHT_TRIGGER_DEADBAND;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_STICK_DEADBAND;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_TARGET_ANGLE_A_RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_TARGET_ANGLE_B_RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.TEST_MODULE_TARGET_ANGLE_X_RADIANS;

@TeleOp(name = "Single Swerve Module Test", group = "Test")
public class SingleSwerveModuleTest extends OpMode {
    private SwerveModule module;
    private DcMotorEx motorA;
    private DcMotorEx motorB;
    private double targetAngleRadians;
    private boolean previousY;

    @Override
    public void init() {
        module = new SwerveModule(
                hardwareMap,
                TEST_MODULE_MOTOR_A,
                TEST_MODULE_MOTOR_B,
                TEST_MODULE_MOTOR_A_INVERTED,
                TEST_MODULE_MOTOR_B_INVERTED
        );

        motorA = hardwareMap.get(DcMotorEx.class, TEST_MODULE_MOTOR_A);
        motorB = hardwareMap.get(DcMotorEx.class, TEST_MODULE_MOTOR_B);
        targetAngleRadians = TEST_MODULE_TARGET_ANGLE_A_RADIANS;

        telemetry.addLine("Configure only two motors:");
        telemetry.addData("motorA", TEST_MODULE_MOTOR_A);
        telemetry.addData("motorB", TEST_MODULE_MOTOR_B);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.y && !previousY) {
            module.resetEncoders();
        }
        previousY = gamepad1.y;

        if (gamepad1.a) {
            targetAngleRadians = TEST_MODULE_TARGET_ANGLE_A_RADIANS;
        } else if (gamepad1.b) {
            targetAngleRadians = TEST_MODULE_TARGET_ANGLE_B_RADIANS;
        } else if (gamepad1.x) {
            targetAngleRadians = TEST_MODULE_TARGET_ANGLE_X_RADIANS;
        } else if (getRightStickMagnitude() > TEST_MODULE_STICK_DEADBAND) {
            targetAngleRadians = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
        }

        double driveSpeedMetersPerSecond =
                -gamepad1.left_stick_y * MAX_DRIVE_SPEED_MPS * TEST_MODULE_MANUAL_POWER_SCALE;
        if (gamepad1.right_trigger > TEST_MODULE_RIGHT_TRIGGER_DEADBAND) {
            driveSpeedMetersPerSecond =
                    MAX_DRIVE_SPEED_MPS * TEST_MODULE_CLOSED_LOOP_DRIVE_POWER * gamepad1.right_trigger;
        }

        module.setState(new SwerveModuleState(
                driveSpeedMetersPerSecond,
                new Rotation2d(targetAngleRadians)
        ));
        sendTelemetry(driveSpeedMetersPerSecond);
    }

    @Override
    public void stop() {
        module.stop();
    }

    private double getRightStickMagnitude() {
        return Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
    }

    private void sendTelemetry(double driveSpeedMetersPerSecond) {
        double angleRadians = module.getAngleRadians();
        double angleError = wrapRadians(targetAngleRadians - angleRadians);

        telemetry.addData("Controls", "LY speed, RS target angle, A/B/X preset angles, RT slow forward, Y reset");
        telemetry.addData("target deg", "%.1f", Math.toDegrees(targetAngleRadians));
        telemetry.addData("angle deg", "%.1f", Math.toDegrees(angleRadians));
        telemetry.addData("angle rad", "%.3f", angleRadians);
        telemetry.addData("angle error deg", "%.1f", Math.toDegrees(angleError));
        telemetry.addData("target speed m/s", "%.2f", driveSpeedMetersPerSecond);
        telemetry.addData("module speed m/s", "%.2f", module.getDriveVelocityMetersPerSecond());
        telemetry.addData("drive meters", "%.3f", module.getDriveDistanceMeters());
        telemetry.addData("motorA ticks", motorA.getCurrentPosition());
        telemetry.addData("motorB ticks", motorB.getCurrentPosition());
        telemetry.addData("module steer ticks = (A+B)/2", "%.1f", module.getSteerTicks());
        telemetry.addData("module drive ticks = (A-B)/2", "%.1f", module.getDriveTicks());
        telemetry.addData("drive power scale", "%.4f", DIFFERENTIAL_DRIVE_POWER_SCALE);
        telemetry.addData("steer power scale", "%.4f", DIFFERENTIAL_STEER_POWER_SCALE);
        telemetry.addData("optimize min speed m/s", "%.3f", MODULE_OPTIMIZE_MIN_SPEED_MPS);
        telemetry.addData("max drive m/s", "%.2f", MAX_DRIVE_SPEED_MPS);
        telemetry.update();
    }

    private double wrapRadians(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
