package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Swerve Calibration", group = "Calibration")
public class SwerveCalibrationOpMode extends LinearOpMode {

    private enum Module {
        FR, FL, BR, BL
    }

    private Module selected = Module.FR;
    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void runOpMode() {
        SwerveDrive swerve = new SwerveDrive(hardwareMap);

        telemetry.addLine("Swerve Calibration Mode Ready");
        telemetry.addLine("Use left stick x to slowly move selected module");
        telemetry.addLine("Rotate wheel until it physically faces front");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleSelection();

            double power = gamepad1.left_stick_x * 0.20;

            swerve.stop();

            switch (selected) {
                case FR:
                    swerve.fr.setDesiredState(0.0, swerve.fr.getWheelAngleDeg() + power * 10.0);
                    swerve.fr.update();
                    break;
                case FL:
                    swerve.fl.setDesiredState(0.0, swerve.fl.getWheelAngleDeg() + power * 10.0);
                    swerve.fl.update();
                    break;
                case BR:
                    swerve.br.setDesiredState(0.0, swerve.br.getWheelAngleDeg() + power * 10.0);
                    swerve.br.update();
                    break;
                case BL:
                    swerve.bl.setDesiredState(0.0, swerve.bl.getWheelAngleDeg() + power * 10.0);
                    swerve.bl.update();
                    break;
            }

            telemetry.addLine("=== SWERVE CALIBRATION ===");
            telemetry.addData("Selected Module", selected.name());
            telemetry.addLine("Turn selected wheel until it faces robot front");
            telemetry.addLine("Then copy RAW angle into SwerveConstants offset");
            telemetry.addLine("");

            addOne("FR_FORWARD_OFFSET_DEG", swerve.fr);
            addOne("FL_FORWARD_OFFSET_DEG", swerve.fl);
            addOne("BR_FORWARD_OFFSET_DEG", swerve.br);
            addOne("BL_FORWARD_OFFSET_DEG", swerve.bl);

            telemetry.update();
        }

        swerve.stop();
    }

    private void handleSelection() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (up && !lastUp) {
            selected = previous(selected);
        }
        if (down && !lastDown) {
            selected = next(selected);
        }

        lastUp = up;
        lastDown = down;
    }

    private Module next(Module m) {
        Module[] vals = Module.values();
        return vals[(m.ordinal() + 1) % vals.length];
    }

    private Module previous(Module m) {
        Module[] vals = Module.values();
        return vals[(m.ordinal() - 1 + vals.length) % vals.length];
    }

    private void addOne(String label, SwerveModule m) {
        telemetry.addLine("[" + m.getName() + "]");
        telemetry.addData(" raw_voltage", "%.4f", m.getAnalogVoltage());
        telemetry.addData(" raw_angle_deg", "%.2f", m.getRawAnalogAngleDeg());
        telemetry.addData(label, "%.2f", m.getRawAnalogAngleDeg());
        telemetry.addLine("");
    }
}