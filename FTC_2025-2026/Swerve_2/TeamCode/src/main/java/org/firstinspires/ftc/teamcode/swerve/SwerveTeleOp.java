package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Swerve TeleOp", group = "Drive")
public class SwerveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwerveTuningData data = SwerveTuningStorage.load();

        SwerveConstants.MAX_TRANSLATION = data.maxTranslation;
        SwerveConstants.MAX_ROTATION = data.maxRotation;
        SwerveConstants.STEER_kP = data.steerKP;
        SwerveConstants.STEER_kD = data.steerKD;
        SwerveConstants.STEER_MAX_POWER = data.steerMaxPower;
        SwerveConstants.STEER_TOLERANCE_DEG = data.steerToleranceDeg;

        SwerveDrive swerve = new SwerveDrive(hardwareMap);

        telemetry.addLine("Swerve initialized");
        telemetry.addData("MAX_TRANSLATION", "%.4f", SwerveConstants.MAX_TRANSLATION);
        telemetry.addData("MAX_ROTATION", "%.4f", SwerveConstants.MAX_ROTATION);
        telemetry.addData("STEER_kP", "%.6f", SwerveConstants.STEER_kP);
        telemetry.addData("STEER_kD", "%.6f", SwerveConstants.STEER_kD);
        telemetry.addData("STEER_MAX_POWER", "%.4f", SwerveConstants.STEER_MAX_POWER);
        telemetry.addData("STEER_TOLERANCE_DEG", "%.4f", SwerveConstants.STEER_TOLERANCE_DEG);
        telemetry.addLine("Init will point all wheels forward");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            swerve.pointAllForward();
            swerve.update();

            telemetry.addLine("Init alignment -> pointing all modules forward");
            addModuleTelemetry(swerve);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double x = applyDeadband(gamepad1.left_stick_x, 0.05) * SwerveConstants.MAX_TRANSLATION;
            double y = applyDeadband(-gamepad1.left_stick_y, 0.05) * SwerveConstants.MAX_TRANSLATION;
            double omega = applyDeadband(gamepad1.right_stick_x, 0.05) * SwerveConstants.MAX_ROTATION;

            if (gamepad1.a) {
                swerve.pointAllForward();
            } else {
                swerve.setRobotCentric(x, y, omega);
            }

            swerve.update();

            telemetry.addData("cmd_x", "%.3f", x);
            telemetry.addData("cmd_y", "%.3f", y);
            telemetry.addData("cmd_omega", "%.3f", omega);
            telemetry.addLine("A -> point all wheels forward");
            telemetry.addLine("");
            addModuleTelemetry(swerve);
            telemetry.update();
        }

        swerve.stop();
    }

    private double applyDeadband(double val, double db) {
        return Math.abs(val) < db ? 0.0 : val;
    }

    private void addModuleTelemetry(SwerveDrive swerve) {
        addOne(swerve.fr);
        addOne(swerve.fl);
        addOne(swerve.br);
        addOne(swerve.bl);
    }

    private void addOne(SwerveModule m) {
        telemetry.addLine("[" + m.getName() + "]");
        telemetry.addData(" raw_analog_voltage", "%.4f", m.getAnalogVoltage());
        telemetry.addData(" angle_now_deg", "%.2f", m.getWheelAngleDeg());
        telemetry.addData(" angle_target_deg", "%.2f", m.getTargetWheelAngleDeg());
        telemetry.addData(" angle_error_deg", "%.2f", m.getAngleErrorDeg());
        telemetry.addData(" drive_target_power", "%.2f", m.getTargetDrivePower());
        telemetry.addData(" steer_power", "%.2f", m.getLastSteerPower());
        telemetry.addData(" drive_motor_rpm", "%.2f", m.getDriveMotorRPM());
        telemetry.addData(" wheel_rpm", "%.2f", m.getWheelRPM());
        telemetry.addData(" wheel_speed_mps", "%.3f", m.getWheelLinearSpeedMps());
        telemetry.addLine("");
    }
}