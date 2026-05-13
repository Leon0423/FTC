package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveDrive {

    public final SwerveModule fr;
    public final SwerveModule fl;
    public final SwerveModule br;
    public final SwerveModule bl;

    private final double L;
    private final double W;
    private final double R;

    public SwerveDrive(HardwareMap hardwareMap) {
        fr = new SwerveModule(
                hardwareMap,
                "FR",
                SwerveConstants.FR_DRIVE, SwerveConstants.FR_DRIVE_REVERSED,
                SwerveConstants.FR_TURN, SwerveConstants.FR_TURN_REVERSED,
                SwerveConstants.FR_ANALOG, SwerveConstants.FR_ANALOG_REVERSED,
                SwerveConstants.FR_FORWARD_OFFSET_DEG
        );

        fl = new SwerveModule(
                hardwareMap,
                "FL",
                SwerveConstants.FL_DRIVE, SwerveConstants.FL_DRIVE_REVERSED,
                SwerveConstants.FL_TURN, SwerveConstants.FL_TURN_REVERSED,
                SwerveConstants.FL_ANALOG, SwerveConstants.FL_ANALOG_REVERSED,
                SwerveConstants.FL_FORWARD_OFFSET_DEG
        );

        br = new SwerveModule(
                hardwareMap,
                "BR",
                SwerveConstants.BR_DRIVE, SwerveConstants.BR_DRIVE_REVERSED,
                SwerveConstants.BR_TURN, SwerveConstants.BR_TURN_REVERSED,
                SwerveConstants.BR_ANALOG, SwerveConstants.BR_ANALOG_REVERSED,
                SwerveConstants.BR_FORWARD_OFFSET_DEG
        );

        bl = new SwerveModule(
                hardwareMap,
                "BL",
                SwerveConstants.BL_DRIVE, SwerveConstants.BL_DRIVE_REVERSED,
                SwerveConstants.BL_TURN, SwerveConstants.BL_TURN_REVERSED,
                SwerveConstants.BL_ANALOG, SwerveConstants.BL_ANALOG_REVERSED,
                SwerveConstants.BL_FORWARD_OFFSET_DEG
        );

        L = SwerveConstants.WHEEL_BASE;
        W = SwerveConstants.TRACK_WIDTH;
        R = Math.hypot(L, W);
    }

    public void setRobotCentric(double x, double y, double omega) {
        double A = x - omega * (L / R);
        double B = x + omega * (L / R);
        double C = y - omega * (W / R);
        double D = y + omega * (W / R);

        double frSpeed = Math.hypot(B, C);
        double flSpeed = Math.hypot(B, D);
        double brSpeed = Math.hypot(A, C);
        double blSpeed = Math.hypot(A, D);

        double frAngle = Math.toDegrees(Math.atan2(B, C));
        double flAngle = Math.toDegrees(Math.atan2(B, D));
        double brAngle = Math.toDegrees(Math.atan2(A, C));
        double blAngle = Math.toDegrees(Math.atan2(A, D));

        double max = Math.max(Math.max(frSpeed, flSpeed), Math.max(brSpeed, blSpeed));
        if (max > 1.0) {
            frSpeed /= max;
            flSpeed /= max;
            brSpeed /= max;
            blSpeed /= max;
        }

        fr.setDesiredState(frSpeed, frAngle);
        fl.setDesiredState(flSpeed, flAngle);
        br.setDesiredState(brSpeed, brAngle);
        bl.setDesiredState(blSpeed, blAngle);
    }

    public void pointAllForward() {
        fr.pointForward();
        fl.pointForward();
        br.pointForward();
        bl.pointForward();
    }

    public void update() {
        fr.update();
        fl.update();
        br.update();
        bl.update();
    }

    public void stop() {
        fr.stop();
        fl.stop();
        br.stop();
        bl.stop();
    }
}