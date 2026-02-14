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
    public static double turningOutputScale = 1.0;  // 輸出縮放

    // === Turning 補助參數 ===
    public static double deadbandDeg = 3.0; // 保留死區，避免小抖動

    // === Drive PID 參數 ===
    public static double driveP = Constants.ModuleConstants.kPDrive;
    public static double driveI = Constants.ModuleConstants.kIDrive;
    public static double driveD = Constants.ModuleConstants.kDDrive;
    public static double driveF = Constants.ModuleConstants.kFDrive;  // 前饋項
    public static double driveOutputScale = 1.0;  // 輸出縮放

    // === 功能開關 ===
    public static boolean enableDrivePID = true;  // 是否啟用 Drive PID
}
