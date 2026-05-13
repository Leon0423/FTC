package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.commands.SwerveJoystickCmd;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

@TeleOp(name = "Swerve_Control", group = "TeleOp")
public class Swerve_Control extends LinearOpMode {

    private SwerveSubsystem swerveSubsystem;

    private GamepadEx driverGamepad;
    private boolean fieldOriented = true;
    private boolean lastA = false;
    private boolean lastLB = false;
    private boolean yLockModeEnabled = false;
    private boolean lastY = false;

    private double targetX = 0;
    private double targetY = 0;
    private double targetHeadingDeg = 0;
    private double lastTime = 0;

    private double speedMultiplier   = 1.0;
    private double turningMultiplier = 1.0;
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadLeft  = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ── Init hardware ────────────────────────────────────────────────
        swerveSubsystem = new SwerveSubsystem(hardwareMap);
        driverGamepad   = new GamepadEx(gamepad1);

        Telemetry dashboardTelemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        SwerveJoystickCmd joystickCmd = new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverGamepad.getLeftY()  * speedMultiplier,
                () -> -driverGamepad.getLeftX()  * speedMultiplier,
                () -> -driverGamepad.getRightX() * turningMultiplier,
                () -> fieldOriented,
                () -> yLockModeEnabled
        );

        // ── Load offset + encoder state ─────────────────────────────────
        SharedPreferences offsetPrefs = AppUtil.getDefContext()
                .getSharedPreferences("SwerveOffsetPrefs", Context.MODE_PRIVATE);
        SharedPreferences trackingPrefs = hardwareMap.appContext
                .getSharedPreferences("SwerveModuleState", Context.MODE_PRIVATE);

        boolean hasOffsets = swerveSubsystem.hasStoredOffsets(offsetPrefs);
        if (hasOffsets) {
            swerveSubsystem.loadOffsets(offsetPrefs);
            swerveSubsystem.initializeAllFromAbsolute();
        } else if (!swerveSubsystem.loadModuleStates(trackingPrefs)) {
            swerveSubsystem.resetAllModuleTracking(true);
        }

        // ── Init phase ───────────────────────────────────────────────────
        boolean zeroed = false; // always align wheels regardless of offset availability
        while (!isStarted() && !isStopRequested()) {
            swerveSubsystem.periodic();

            if (!zeroed) {
                zeroed = swerveSubsystem.alignAllModulesToZero();
                if (zeroed) {
                    swerveSubsystem.resetAllAccumulators();
                }
            }

            dashboardTelemetry.addData("Status",
                    zeroed ? "Init: Ready" : "Init: Aligning wheels to 0...");
            dashboardTelemetry.addData("Offset Mode", hasOffsets ? "YES" : "NO (fallback)");
            dashboardTelemetry.update();
            idle();
        }

        if (isStopRequested()) {
            swerveSubsystem.stopModules();
            return;
        }

        // ── Start ────────────────────────────────────────────────────────
        swerveSubsystem.zeroHeading();
        lastTime = getRuntime();

        // ── Main loop ────────────────────────────────────────────────────
        try {
            while (opModeIsActive()) {

                driverGamepad.readButtons();

                // Left Bumper: toggle field-oriented
                boolean lb = driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER);
                if (lb && !lastLB) fieldOriented = !fieldOriented;
                lastLB = lb;

                // A: reset heading only
                boolean a = driverGamepad.getButton(GamepadKeys.Button.A);
                if (a && !lastA) {
                    swerveSubsystem.zeroHeading();
                    targetHeadingDeg = 0;
                }
                lastA = a;

                // Y: toggle X-lock mode
                boolean y = driverGamepad.getButton(GamepadKeys.Button.Y);
                if (y && !lastY) yLockModeEnabled = !yLockModeEnabled;
                lastY = y;

                // DPAD UP/DOWN: speed multiplier
                boolean dpadUp   = driverGamepad.getButton(GamepadKeys.Button.DPAD_UP);
                boolean dpadDown = driverGamepad.getButton(GamepadKeys.Button.DPAD_DOWN);
                if (dpadUp   && !lastDpadUp)   speedMultiplier = Math.min(1.0, speedMultiplier + 0.1);
                if (dpadDown && !lastDpadDown) speedMultiplier = Math.max(0.1, speedMultiplier - 0.1);
                lastDpadUp   = dpadUp;
                lastDpadDown = dpadDown;

                // DPAD LEFT/RIGHT: turning multiplier
                boolean dpadLeft  = driverGamepad.getButton(GamepadKeys.Button.DPAD_LEFT);
                boolean dpadRight = driverGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT);
                if (dpadRight && !lastDpadRight) turningMultiplier = Math.min(1.0, turningMultiplier + 0.1);
                if (dpadLeft  && !lastDpadLeft)  turningMultiplier = Math.max(0.1, turningMultiplier - 0.1);
                lastDpadLeft  = dpadLeft;
                lastDpadRight = dpadRight;

                // ── Target-pose integration (for dashboard) ──────────────
                double now = getRuntime();
                double dt  = Math.max(0, now - lastTime);
                lastTime = now;

                double rawX    = -driverGamepad.getLeftY();
                double rawY    = -driverGamepad.getLeftX();
                double rawTurn = -driverGamepad.getRightX();

                rawX    = Math.abs(rawX)    > 0.05 ? rawX    : 0;
                rawY    = Math.abs(rawY)    > 0.05 ? rawY    : 0;
                rawTurn = Math.abs(rawTurn) > 0.05 ? rawTurn : 0;

                double xSpeed    = rawX    * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                double ySpeed    = rawY    * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
                double turnSpeed = rawTurn * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

                double headingRad = Math.toRadians(targetHeadingDeg);
                double fieldX = xSpeed, fieldY = ySpeed;
                if (!fieldOriented) {
                    double cos = Math.cos(headingRad), sin = Math.sin(headingRad);
                    fieldX = xSpeed * cos - ySpeed * sin;
                    fieldY = xSpeed * sin + ySpeed * cos;
                }

                targetX          += fieldX * dt;
                targetY          += fieldY * dt;
                targetHeadingDeg += Math.toDegrees(turnSpeed * dt);

                swerveSubsystem.setTargetPose(targetX, targetY, targetHeadingDeg);

                // ── Execute drive command ────────────────────────────────
                joystickCmd.execute();

                // ── Update subsystem + telemetry ─────────────────────────
                swerveSubsystem.periodic();
                swerveSubsystem.updateTelemetry(dashboardTelemetry);

                dashboardTelemetry.addData("Mode",
                        fieldOriented ? "Field-Oriented" : "Robot-Oriented");
                dashboardTelemetry.addData("Y-Lock Mode",
                        yLockModeEnabled ? "LOCK (idle -> X)" : "UNLOCK");
                dashboardTelemetry.addData("Heading", "%.1f", swerveSubsystem.getHeading());

                Pose2d pose = swerveSubsystem.getPose();
                dashboardTelemetry.addData("Position",
                        "X:%.2f Y:%.2f", pose.getX(), pose.getY());
                dashboardTelemetry.addData("Target",
                        "X:%.2f Y:%.2f H:%.1f", targetX, targetY, targetHeadingDeg);

                dashboardTelemetry.addData("Drive RPM",
                        "FL:%.0f FR:%.0f BL:%.0f BR:%.0f",
                        swerveSubsystem.getFrontLeft().getDriveRPM(),
                        swerveSubsystem.getFrontRight().getDriveRPM(),
                        swerveSubsystem.getBackLeft().getDriveRPM(),
                        swerveSubsystem.getBackRight().getDriveRPM());

                dashboardTelemetry.addData("Turning (deg)",
                        "FL:%.1f FR:%.1f BL:%.1f BR:%.1f",
                        Math.toDegrees(swerveSubsystem.getFrontLeft().getTurningPosition()),
                        Math.toDegrees(swerveSubsystem.getFrontRight().getTurningPosition()),
                        Math.toDegrees(swerveSubsystem.getBackLeft().getTurningPosition()),
                        Math.toDegrees(swerveSubsystem.getBackRight().getTurningPosition()));

                dashboardTelemetry.addData("Accum Wheel (deg)",
                        "FL:%.1f FR:%.1f BL:%.1f BR:%.1f",
                        swerveSubsystem.getFrontLeft().getAccumulatedWheelDeg(),
                        swerveSubsystem.getFrontRight().getAccumulatedWheelDeg(),
                        swerveSubsystem.getBackLeft().getAccumulatedWheelDeg(),
                        swerveSubsystem.getBackRight().getAccumulatedWheelDeg());

                dashboardTelemetry.addData("Speed",   "%.0f%%", speedMultiplier   * 100);
                dashboardTelemetry.addData("Turning", "%.0f%%", turningMultiplier * 100);
                dashboardTelemetry.addData("Status", "Running");
                dashboardTelemetry.update();
            }
        } finally {
            // ── Shutdown: save encoder state ─────────────────────────────
            swerveSubsystem.periodic();
            swerveSubsystem.stopModules();

            SharedPreferences.Editor editor = trackingPrefs.edit();
            swerveSubsystem.saveModuleStates(editor);
            editor.apply();
        }
    }
}
