package org.firstinspires.ftc.teamcode;

// 引入必要的函式庫
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import static org.firstinspires.ftc.teamcode.Units.inchesToMeters;

/**
 * 全域常數類別
 * 包含機器人所有子系統的配置參數
 * 用於集中管理機器人硬體配置和控制參數
 */
public final class Constants {
    /**
     * 飛輪模組常數類別
     * 定義單一飛輪模組的物理特性和控制參數
     */
    public static final class ModuleConstants{
        // === 機械參數 ===
        public static final double kWheelDiameterMeters = 0.08; // 輪子直徑(公尺) - 調整輪子直徑(單位: m)
        public static final double kDriveMotorGearRatio = 1;    // 驅動馬達齒輪比 - 調整驅動馬達轉速比
        public static final double kTurningMotorGearRatio = 1 ; // 轉向馬達齒輪比 - 調整轉向馬達轉速比

        // === 編碼器換算係數 ===
        // 將馬達旋轉數轉換為線性距離的係數
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        // 將轉向馬達旋轉數轉換為弧度的係數
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        // 將驅動馬達RPM轉換為每秒公尺的係數
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        // 將轉向馬達RPM轉換為每秒弧度的係數
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        // === 轉向 PID 控制器參數 ===
        public static final double kPTurning = 0.45;  // P 係數：反應速度，過大會震盪
        public static final double kITurning = 0.0;   // I 係數：消除穩態誤差
        public static final double kDTurning = 0.0;   // D 係數：抑制震盪
        public static final double kTurningOutputScale = 1.0; // 轉向輸出縮放

        // 轉向輸出與感測穩定化
        public static final double kTurningDeadbandDeg = 3.0;     // 死區角度 (度)
        public static final double kTurningMinOutput = 0.05;      // 轉向馬達最小輸出 (克服靜摩擦)
        public static final double kTurningMaxJumpDeg = 45.0;     // 單次允許的角度跳變上限 (度)
        public static final double kTurningMaxTransitionOutput = 0.7; // 過渡期間最大輸出

        // === 驅動 PID 控制器參數 ===
        public static final double kPDrive = 0.1;   // P 係數：速度響應
        public static final double kIDrive = 0.0;   // I 係數：穩態誤差
        public static final double kDDrive = 0.0;   // D 係數：抑制震盪
        public static final double kFDrive = 0.0;   // F 係數：前饋補償
        public static final double kDriveOutputScale = 1.0; // 驅動輸出縮放
        public static final boolean kEnableDrivePID = false; // 是否啟用 Drive PID

    }

    /**
     * 飛輪驅動系統常數類別
     * 定義整個飛輪驅動系統的硬體配置和運動參數
     */
    public static final class DriveConstants {
        // === 機器人幾何參數 ===
        public static final double kTrackWidth = 0.33;  // 左右輪距 (公尺)     // TODO: 調整
        public static final double kWheelBase = 0.33;   // 前後輪距 (公尺)     // TODO: 調整

        // 飛輪驅動運動學計算，定義各輪子相對於機器人中心的位置
        // 順序：前左、前右、後左、後右
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // 前左輪位置   // TODO: 調整
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // 前右輪位置   // TODO: 調整
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // 後左輪位置   // TODO: 調整
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // 後右輪位置   // TODO: 調整

        // === 驅動馬達硬體名稱 ===
        public static final String kFrontLeftDriveMotorName = "FL";   // 前左驅動馬達
        public static final String kBackLeftDriveMotorName = "BL";    // 後左驅動馬達
        public static final String kFrontRightDriveMotorName = "FR";  // 前右驅動馬達
        public static final String kBackRightDriveMotorName = "BR";   // 後右驅動馬達

        // === 轉向馬達硬體名稱 ===
        public static final String kFrontLeftTurningMotorName = "FLTurn";   // 前左轉向馬達
        public static final String kBackLeftTurningMotorName = "BLTurn";    // 後左轉向馬達
        public static final String kFrontRightTurningMotorName = "FRTurn";  // 前右轉向馬達
        public static final String kBackRightTurningMotorName = "BRTurn";   // 後右轉向馬達

        // === 轉向編碼器方向設定 ===
        public static final boolean kFrontLeftTurningEncoderReversed = true;   // 前左轉向編碼器反向
        public static final boolean kBackLeftTurningEncoderReversed = true;    // 後左轉向編碼器反向
        public static final boolean kFrontRightTurningEncoderReversed = true;  // 前右轉向編碼器反向
        public static final boolean kBackRightTurningEncoderReversed = true;   // 後右轉向編碼器反向

        // === 驅動編碼器方向設定 ===
        public static final boolean kFrontLeftDriveEncoderReversed = false;   // 前左驅動編碼器反向
        public static final boolean kBackLeftDriveEncoderReversed = false;    // 後左驅動編碼器反向
        public static final boolean kFrontRightDriveEncoderReversed = false;  // 前右驅動編碼器反向
        public static final boolean kBackRightDriveEncoderReversed = false;   // 後右驅動編碼器反向

        // === 絕對編碼器硬體名稱 ===
        // 使用與 servo 相同的命名方式，便於硬體配置管理
        public static final String kFrontLeftAbsoluteEncoderName = "FLEncoder";   // 前左絕對編碼器
        public static final String kFrontRightAbsoluteEncoderName = "FREncoder";  // 前右絕對編碼器
        public static final String kBackLeftAbsoluteEncoderName = "BLEncoder";    // 後左絕對編碼器
        public static final String kBackRightAbsoluteEncoderName = "BREncoder";   // 後右絕對編碼器

        // === 絕對編碼器方向設定 ===
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;   // 前左絕對編碼器反向
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;    // 後左絕對編碼器反向
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;  // 前右絕對編碼器反向
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;   // 後右絕對編碼器反向

        // === 絕對編碼器偏移角度 (度數) ===
        // 用於校正輪子的初始方向，使用 SwerveModuleTuner 測量正確的偏移值
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 116.6;  // 前左輪偏移角度
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = 186.4; // 前右輪偏移角度
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = 188.7;   // 後左輪偏移角度
        public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = 4.7;    // 後右輪偏移角度

        // === 絕對編碼器偏移角度 (弧度) - 自動計算 ===
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(kFrontLeftDriveAbsoluteEncoderOffsetDeg);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(kBackLeftDriveAbsoluteEncoderOffsetDeg);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(kFrontRightDriveAbsoluteEncoderOffsetDeg);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(kBackRightDriveAbsoluteEncoderOffsetDeg);

        // === 機器人物理性能極限 ===
        public static final double kPhysicalMaxSpeedMetersPerSecond = 11.6;                       // 機器人最大線速度 (公尺/秒)
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;    // 機器人最大角速度 (弧度/秒)

        // === 手動控制性能限制 ===
        // 建議調整後進行測試以確保良好的操控手感
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;                    // 手動控制最大線速度
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;    // 手動控制最大角速度
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;                                                 // 最大線性加速度
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;                                          // 最大角加速度

        // === IMU 慣性測量單元配置 ===
        // 必須根據 Control Hub 的實際安裝方向進行設定
        public static final RevHubOrientationOnRobot.LogoFacingDirection kImuLogoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;        // REV 標誌朝向
        public static final RevHubOrientationOnRobot.UsbFacingDirection kImuUsbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;    // USB 端口朝向
    }

    /**
     * 操作者介面常數類別
     * 定義手把和輸入裝置的相關參數
     */
    public static final class OIConstants {
        public static final double kDeadband = 0.05;  // 手把搖桿死區範圍，避免微小抖動影響
    }
}
