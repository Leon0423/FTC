package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Live-tunable parameters exposed on FTC Dashboard.
 */
@Config
public class TuningConfig {
    // === Turning PID 參數 ===
    public static double turningP = Constants.ModuleConstants.kPTurning;
    public static double turningI = Constants.ModuleConstants.kITurning;
    public static double turningD = Constants.ModuleConstants.kDTurning;
    public static double turningOutputScale = Constants.ModuleConstants.kTurningOutputScale;

    // === Turning 補助參數 ===
    public static double deadbandDeg = Constants.ModuleConstants.kTurningDeadbandDeg;
    public static double minOutput = Constants.ModuleConstants.kTurningMinOutput;
    public static double maxJumpDeg = Constants.ModuleConstants.kTurningMaxJumpDeg;
    public static double maxTransitionOutput = Constants.ModuleConstants.kTurningMaxTransitionOutput;

    // === Drive PID 參數 ===
    public static double driveP = Constants.ModuleConstants.kPDrive;
    public static double driveI = Constants.ModuleConstants.kIDrive;
    public static double driveD = Constants.ModuleConstants.kDDrive;
    public static double driveF = Constants.ModuleConstants.kFDrive;
    public static double driveOutputScale = Constants.ModuleConstants.kDriveOutputScale;

    // === 功能開關 ===
    public static boolean enableDrivePID = Constants.ModuleConstants.kEnableDrivePID;
}
