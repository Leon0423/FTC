package org.firstinspires.ftc.teamcode.Tuning;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

/**
 * Pinpoint 快速檢查：
 * - 推車觀察 X/Y/Heading 是否符合預期
 * - A: 只歸零 Heading（不改 X/Y）
 * - B: 重設 Pose 到 (0,0,0)
 */
@TeleOp(name = "6. PinpointQuickCheck", group = "Tuning")
public class _6_PinpointQuickCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwerveSubsystem swerve = new SwerveSubsystem(hardwareMap);

        boolean lastA = false;
        boolean lastB = false;

        telemetry.addLine("Pinpoint Quick Check");
        telemetry.addLine("Push robot by hand. Check signs of X/Y/Heading.");
        telemetry.addLine("A: zeroHeading only, B: reset odometry to (0,0,0)");
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

            telemetry.addLine("=== Pinpoint Status ===");
            telemetry.addData("Pose", "X: %.3f m, Y: %.3f m", pose.getX(), pose.getY());
            telemetry.addData("Heading", "%.1f deg", heading);
            telemetry.addLine("Expected: forward -> X+, right strafe -> check your team convention");
            telemetry.addData("Controls", "A=zeroHeading, B=resetPose");
            telemetry.update();
        }

        swerve.stopModules();
    }
}

