package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Swerve Tuning TeleOp", group = "Drive")
public class SwerveTuningTeleOp extends LinearOpMode {

    private enum Param {
        MAX_TRANSLATION,
        MAX_ROTATION,
        STEER_kP,
        STEER_kD,
        STEER_MAX_POWER,
        STEER_TOLERANCE_DEG
    }

    private Param selectedParam = Param.MAX_TRANSLATION;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    private boolean steerOnlyMode = false;
    private boolean saveSuccess = false;
    private String saveMessage = "Not saved yet";

    @Override
    public void runOpMode() {
        SwerveTuningData loaded = SwerveTuningStorage.load();
        SwerveConstants.MAX_TRANSLATION = loaded.maxTranslation;
        SwerveConstants.MAX_ROTATION = loaded.maxRotation;
        SwerveConstants.STEER_kP = loaded.steerKP;
        SwerveConstants.STEER_kD = loaded.steerKD;
        SwerveConstants.STEER_MAX_POWER = loaded.steerMaxPower;
        SwerveConstants.STEER_TOLERANCE_DEG = loaded.steerToleranceDeg;

        SwerveDrive swerve = new SwerveDrive(hardwareMap);

        telemetry.addLine("Swerve tuning mode ready");
        telemetry.addLine("Init: aligning all wheels forward");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            swerve.pointAllForward();
            swerve.update();

            telemetry.addLine("Waiting for start...");
            telemetry.addData("Loaded MAX_TRANSLATION", "%.4f", SwerveConstants.MAX_TRANSLATION);
            telemetry.addData("Loaded MAX_ROTATION", "%.4f", SwerveConstants.MAX_ROTATION);
            telemetry.addData("Loaded STEER_kP", "%.6f", SwerveConstants.STEER_kP);
            telemetry.addData("Loaded STEER_kD", "%.6f", SwerveConstants.STEER_kD);
            telemetry.addData("Loaded STEER_MAX_POWER", "%.4f", SwerveConstants.STEER_MAX_POWER);
            telemetry.addData("Loaded STEER_TOLERANCE_DEG", "%.4f", SwerveConstants.STEER_TOLERANCE_DEG);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleParamSelection();
            handleParamAdjustment();
            handleModeButtons(swerve);
            handleSaveButton();

            double x = applyDeadband(gamepad1.left_stick_x, 0.05) * SwerveConstants.MAX_TRANSLATION;
            double y = applyDeadband(-gamepad1.left_stick_y, 0.05) * SwerveConstants.MAX_TRANSLATION;
            double omega = applyDeadband(gamepad1.right_stick_x, 0.05) * SwerveConstants.MAX_ROTATION;

            if (steerOnlyMode) {
                x = 0.0;
                y = 0.0;
                omega = 0.0;
            }

            swerve.setRobotCentric(x, y, omega);
            swerve.update();

            telemetry.addLine("=== SWERVE TUNING MODE ===");
            telemetry.addData("Selected Param", selectedParam.name());
            telemetry.addData("steerOnlyMode", steerOnlyMode);
            telemetry.addData("saveSuccess", saveSuccess);
            telemetry.addData("saveMessage", saveMessage);
            telemetry.addLine("");

            telemetry.addData("MAX_TRANSLATION", "%.4f", SwerveConstants.MAX_TRANSLATION);
            telemetry.addData("MAX_ROTATION", "%.4f", SwerveConstants.MAX_ROTATION);
            telemetry.addData("STEER_kP", "%.6f", SwerveConstants.STEER_kP);
            telemetry.addData("STEER_kD", "%.6f", SwerveConstants.STEER_kD);
            telemetry.addData("STEER_MAX_POWER", "%.4f", SwerveConstants.STEER_MAX_POWER);
            telemetry.addData("STEER_TOLERANCE_DEG", "%.4f", SwerveConstants.STEER_TOLERANCE_DEG);
            telemetry.addLine("");

            telemetry.addLine("Controls:");
            telemetry.addLine("dpad up/down = select param");
            telemetry.addLine("dpad left/right = adjust param");
            telemetry.addLine("RB = large step");
            telemetry.addLine("A = point all wheels forward");
            telemetry.addLine("B = steer-only mode");
            telemetry.addLine("X = normal drive mode");
            telemetry.addLine("Y = SAVE tuning");
            telemetry.addLine("");

            telemetry.addData("cmd_x", "%.3f", x);
            telemetry.addData("cmd_y", "%.3f", y);
            telemetry.addData("cmd_omega", "%.3f", omega);
            telemetry.addLine("");

            addModuleTelemetry(swerve);
            telemetry.update();
        }

        swerve.stop();
    }

    private void handleParamSelection() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadUp && !lastDpadUp) {
            selectedParam = previousParam(selectedParam);
        }
        if (dpadDown && !lastDpadDown) {
            selectedParam = nextParam(selectedParam);
        }

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
    }

    private void handleParamAdjustment() {
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        double smallStep = getSmallStep(selectedParam);
        double largeStep = getLargeStep(selectedParam);
        double step = gamepad1.right_bumper ? largeStep : smallStep;

        if (dpadRight && !lastDpadRight) {
            adjustSelected(step);
        }
        if (dpadLeft && !lastDpadLeft) {
            adjustSelected(-step);
        }

        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
    }

    private void handleModeButtons(SwerveDrive swerve) {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        if (a && !lastA) {
            swerve.pointAllForward();
        }
        if (b && !lastB) {
            steerOnlyMode = true;
        }
        if (x && !lastX) {
            steerOnlyMode = false;
        }

        lastA = a;
        lastB = b;
        lastX = x;
    }

    private void handleSaveButton() {
        boolean y = gamepad1.y;

        if (y && !lastY) {
            try {
                SwerveTuningData data = new SwerveTuningData();
                data.maxTranslation = SwerveConstants.MAX_TRANSLATION;
                data.maxRotation = SwerveConstants.MAX_ROTATION;
                data.steerKP = SwerveConstants.STEER_kP;
                data.steerKD = SwerveConstants.STEER_kD;
                data.steerMaxPower = SwerveConstants.STEER_MAX_POWER;
                data.steerToleranceDeg = SwerveConstants.STEER_TOLERANCE_DEG;

                SwerveTuningStorage.save(data);
                saveSuccess = true;
                saveMessage = "Saved";
            } catch (Exception e) {
                saveSuccess = false;
                saveMessage = "Save failed";
            }
        }

        lastY = y;
    }

    private void adjustSelected(double delta) {
        switch (selectedParam) {
            case MAX_TRANSLATION:
                SwerveConstants.MAX_TRANSLATION = clamp(SwerveConstants.MAX_TRANSLATION + delta, 0.0, 1.0);
                break;
            case MAX_ROTATION:
                SwerveConstants.MAX_ROTATION = clamp(SwerveConstants.MAX_ROTATION + delta, 0.0, 1.0);
                break;
            case STEER_kP:
                SwerveConstants.STEER_kP = clamp(SwerveConstants.STEER_kP + delta, 0.0, 1.0);
                break;
            case STEER_kD:
                SwerveConstants.STEER_kD = clamp(SwerveConstants.STEER_kD + delta, 0.0, 1.0);
                break;
            case STEER_MAX_POWER:
                SwerveConstants.STEER_MAX_POWER = clamp(SwerveConstants.STEER_MAX_POWER + delta, 0.0, 1.0);
                break;
            case STEER_TOLERANCE_DEG:
                SwerveConstants.STEER_TOLERANCE_DEG = clamp(SwerveConstants.STEER_TOLERANCE_DEG + delta, 0.0, 30.0);
                break;
        }
    }

    private Param nextParam(Param current) {
        Param[] values = Param.values();
        return values[(current.ordinal() + 1) % values.length];
    }

    private Param previousParam(Param current) {
        Param[] values = Param.values();
        return values[(current.ordinal() - 1 + values.length) % values.length];
    }

    private double getSmallStep(Param param) {
        switch (param) {
            case MAX_TRANSLATION:
            case MAX_ROTATION:
            case STEER_MAX_POWER:
                return 0.02;
            case STEER_kP:
                return 0.001;
            case STEER_kD:
                return 0.0001;
            case STEER_TOLERANCE_DEG:
                return 0.2;
        }
        return 0.01;
    }

    private double getLargeStep(Param param) {
        switch (param) {
            case MAX_TRANSLATION:
            case MAX_ROTATION:
            case STEER_MAX_POWER:
                return 0.05;
            case STEER_kP:
                return 0.003;
            case STEER_kD:
                return 0.0005;
            case STEER_TOLERANCE_DEG:
                return 0.5;
        }
        return 0.05;
    }

    private double applyDeadband(double val, double db) {
        return Math.abs(val) < db ? 0.0 : val;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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