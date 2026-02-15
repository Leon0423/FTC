package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Constants;

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

    // === Drive PID 參數 ===
    public static double driveP = Constants.ModuleConstants.kPDrive;
    public static double driveI = Constants.ModuleConstants.kIDrive;
    public static double driveD = Constants.ModuleConstants.kDDrive;
    public static double driveF = Constants.ModuleConstants.kFDrive;
    public static double driveOutputScale = Constants.ModuleConstants.kDriveOutputScale;

    // === 功能開關 ===
    public static boolean enableDrivePID = Constants.ModuleConstants.kEnableDrivePID;

    // ===== Teleop Acceleration Limits =====
    public static double teleDriveMaxAccel = Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    public static double teleDriveMaxAngularAccel = Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;


}
