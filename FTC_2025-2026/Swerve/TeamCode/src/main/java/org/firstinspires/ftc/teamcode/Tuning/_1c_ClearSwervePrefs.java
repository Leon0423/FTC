package org.firstinspires.ftc.teamcode.Tuning;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@TeleOp(name = "1c. ClearSwervePrefs", group = "Tuning")
public class _1c_ClearSwervePrefs extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Ready to clear persisted swerve tracking values.");
        telemetry.addLine("This clears Control Hub stored wheel-angle accumulation.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        SharedPreferences prefs = AppUtil.getDefContext()
                .getSharedPreferences("SwerveModulePersistentAngles", Context.MODE_PRIVATE);
        prefs.edit().clear().apply();

        SharedPreferences offsetPrefs = AppUtil.getDefContext()
                .getSharedPreferences("SwerveOffsetPrefs", Context.MODE_PRIVATE);
        offsetPrefs.edit().clear().apply();

        telemetry.addLine("Cleared: SwerveModulePersistentAngles");
        telemetry.addLine("Cleared: SwerveOffsetPrefs");
        telemetry.update();

        // Keep message visible briefly before exit.
        sleep(2000);
    }
}

