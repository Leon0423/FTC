package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live-tunable parameters exposed on FTC Dashboard.
 *
 * 參數命名使用前綴來控制 Dashboard 顯示順序：
 * - 1_ : Turning PID 參數
 * - 2_ : Turning 補助參數
 * - 3_ : Drive PID 參數
 * - 4_ : 功能開關
 * - 5_ : Teleop 加速度限制
 */
@Config
public class TuningConfig {
    // === 1. Turning PID 參數 ===
    public static double _1a_turningP = Constants.ModuleConstants.kPTurning;
    public static double _1b_turningI = Constants.ModuleConstants.kITurning;
    public static double _1c_turningD = Constants.ModuleConstants.kDTurning;
    public static double _1d_turningOutputScale = Constants.ModuleConstants.kTurningOutputScale;

    // === 2. Turning 補助參數 ===
    public static double _2a_deadbandDeg = Constants.ModuleConstants.kTurningDeadbandDeg;

    // === 3. Drive PID 參數 ===
    public static double _3a_driveP = Constants.ModuleConstants.kPDrive;
    public static double _3b_driveI = Constants.ModuleConstants.kIDrive;
    public static double _3c_driveD = Constants.ModuleConstants.kDDrive;
    public static double _3d_driveF = Constants.ModuleConstants.kFDrive;
    public static double _3e_driveOutputScale = Constants.ModuleConstants.kDriveOutputScale;

    // === 4. 功能開關 ===
    public static boolean _4a_enableDrivePID = Constants.ModuleConstants.kEnableDrivePID;

    // === 5. Teleop 加速度限制 ===
    public static double _5a_teleDriveMaxAccel = Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond;
    public static double _5b_teleDriveMaxAngularAccel = Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

    // ===== 便利存取方法 (讓其他程式碼不需要改變) =====
    public static double turningP() { return _1a_turningP; }
    public static double turningI() { return _1b_turningI; }
    public static double turningD() { return _1c_turningD; }
    public static double turningOutputScale() { return _1d_turningOutputScale; }
    public static double deadbandDeg() { return _2a_deadbandDeg; }
    public static double driveP() { return _3a_driveP; }
    public static double driveI() { return _3b_driveI; }
    public static double driveD() { return _3c_driveD; }
    public static double driveF() { return _3d_driveF; }
    public static double driveOutputScale() { return _3e_driveOutputScale; }
    public static boolean enableDrivePID() { return _4a_enableDrivePID; }
    public static double teleDriveMaxAccel() { return _5a_teleDriveMaxAccel; }
    public static double teleDriveMaxAngularAccel() { return _5b_teleDriveMaxAngularAccel; }
}
