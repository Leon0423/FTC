package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Live-tunable parameters exposed on FTC Dashboard.
 */
@Config
public class TuningConfig {
    // Turning PID gains for all swerve modules; tweak in Dashboard "Config" tab.
    public static double turningP = Constants.ModuleConstants.kPTurning;
    public static double turningI = Constants.ModuleConstants.kITurning;
    public static double turningD = Constants.ModuleConstants.kDTurning;

    // Optional output scale if you need to soften/boost servo power.
    public static double turningOutputScale = 1.0;
}

