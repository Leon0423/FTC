package org.firstinspires.ftc.teamcode.Tuning;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * Pinpoint 快速檢查：
 * - 推車觀察 X/Y/Heading 是否符合預期
 * - 直接檢查 Pinpoint 連線狀態與原始值
 * - A: 只歸零 Heading（不改 X/Y）
 * - B: 重設 Pose 到 (0,0,0)
 */
@TeleOp(name = "6. PinpointQuickCheck", group = "Tuning")
public class _6_PinpointQuickCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwerveSubsystem swerve = new SwerveSubsystem(hardwareMap);

        // 直接嘗試取得 Pinpoint 以檢查連線
        GoBildaPinpointDriver pinpointDirect = null;
        boolean pinpointConnected = false;
        if (DriveConstants.USING_PINPOINT) {
            try {
                pinpointDirect = hardwareMap.get(GoBildaPinpointDriver.class, DriveConstants.kPinpointName);
                pinpointConnected = true;
            } catch (Exception e) {
                pinpointConnected = false;
            }
        }

        boolean lastA = false;
        boolean lastB = false;

        telemetry.addLine("Pinpoint Quick Check");
        telemetry.addLine("Push robot by hand. Check signs of X/Y/Heading.");
        telemetry.addLine("A: zeroHeading only, B: reset pose to (0,0,0)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !lastA) {
                swerve.zeroHeading();
            }
            if (b && !lastB) {
                swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            }
            lastA = a;
            lastB = b;

            swerve.periodic();

            Pose2d pose = swerve.getPose();
            double heading = swerve.getHeading();
            boolean usingPinpoint = DriveConstants.USING_PINPOINT;

            telemetry.addLine("=== Pinpoint Connection ===");
            telemetry.addData("USING_PINPOINT", usingPinpoint);
            telemetry.addData("Pinpoint Connected", pinpointConnected ? "YES" : "NO (check I2C/name)");

            if (pinpointConnected && pinpointDirect != null) {
                try {
                    pinpointDirect.update();
                    Pose2D rawPose = pinpointDirect.getPosition();
                    telemetry.addLine("--- Pinpoint Raw Values ---");
                    telemetry.addData("Raw X", "%.3f m", rawPose.getX(DistanceUnit.METER));
                    telemetry.addData("Raw Y", "%.3f m", rawPose.getY(DistanceUnit.METER));
                    telemetry.addData("Raw Heading", "%.1f deg", rawPose.getHeading(AngleUnit.DEGREES));
                } catch (Exception e) {
                    telemetry.addLine("Error reading Pinpoint: " + e.getMessage());
                }
            }

            telemetry.addLine("=== Pose from SwerveSubsystem ===");
            telemetry.addData("Pose Source", usingPinpoint ? "PINPOINT (expected)" : "ODOMETRY");
            telemetry.addData("Pose", "X: %.3f m, Y: %.3f m", pose.getX(), pose.getY());
            telemetry.addData("Heading", "%.1f deg", heading);

            if (usingPinpoint && !pinpointConnected) {
                telemetry.addLine("⚠ Pinpoint not connected! Check:");
                telemetry.addLine("  1. kPinpointName = \"" + DriveConstants.kPinpointName + "\"");
                telemetry.addLine("  2. I2C cable & port");
                telemetry.addLine("  3. Pinpoint I2C address");
            }

            telemetry.addLine("Expected: forward -> X+, right strafe -> Y+");
            telemetry.addData("Controls", "A=zeroHeading, B=resetPose");
            telemetry.update();
        }

        swerve.stopModules();
    }
}

