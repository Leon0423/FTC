package org.firstinspires.ftc.teamcode;

// 引入必要的函式庫
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import static org.firstinspires.ftc.teamcode.Tuning.Units.inchesToMeters;

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
        public static final double kWheelDiameterMeters = 0.058; // TODO 輪子直徑(公尺)

        // 驅動馬達外部齒輪減速比：輪子每轉一圈，馬達輸出軸需轉幾圈。
        // 例：外部 1:8.8 減速 → kDriveMotorGearRatio = 1.0 / 8.8 ≈ 0.113636
        public static final double kDriveMotorGearRatio = 0.113636; // TODO 確認外部齒輪比

        // 驅動馬達編碼器每轉脈衝數（ticks per revolution，在馬達輸出軸量）
        // 使用馬達：goBILDA 5000-0002-4008（直驅，無內部行星齒輪箱）
        // 規格：Encoder Countable Events Per Revolution (Output Shaft) = 28
        // FTC SDK getVelocity() 回傳 ticks/sec，除以此值得到輸出軸轉速（rev/sec）。
        public static final double kDriveEncoderTicksPerRev = 28.0;

        // CRServo 轉 2.5 圈 = 輪角 360°，因此輪角/servo角 比例為 1/2.5。
        public static final double kServoTurnsPerWheelTurn = 2.5;
        public static final double kTurningMotorGearRatio = 1.0 / kServoTurnsPerWheelTurn;

        // === 編碼器換算係數（自動計算，勿手動改） ===
        // ticks/sec ÷ ticks/rev × 齒輪比 × 周長 = 輪速 m/s
        public static final double kDriveEncoderRot2Meter =
                (kDriveMotorGearRatio / kDriveEncoderTicksPerRev) * Math.PI * kWheelDiameterMeters;
        // 將轉向馬達旋轉數轉換為弧度的係數
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

        // === 轉向 PID 控制器參數 ===
        public static final double kPTurning = 0.8;  // TODO: 從 0.8 起調，反應太慢可加到 1.2
        public static final double kITurning = 0.02; // 消除穩態誤差，過大會積分飽和
        public static final double kDTurning = 0.05; // 抑制 kP 提高後的超調，可從 0.03 起調
        public static final double kTurningOutputScale = 1.0;

        // 轉向輸出與感測穩定化
        public static final double kTurningDeadbandDeg = 0.0;     // 精準模式：不設死區
        // CRServo 靜摩擦補償：超過 deadband 後保證最小輸出功率。
        // TODO: 實機測試 CRServo 剛好能動的最低功率，填入此值（建議初值 0.05）。
        public static final double kTurningMinOutput = 0.05;

        // === 驅動 PID 控制器參數 ===
        // 調整於 2026-03-16：解決馬達力矩過大轉不動問題
        // 啟用PID以提供速度反饋控制，加強低速時的扭矩以克服卡住
        public static final double kPDrive = 0.0;  // TODO P 係數：降低以減少過度響應，防止抖動
        public static final double kIDrive = 0.0;  // TODO I 係數：增加至 0.01 以穩定低速轉動
        public static final double kDDrive = 0.0; // TODO D 係數：添加阻尼以穩定控制
        public static final double kFDrive = 0.0;  // TODO F 係數：增加前饋至 0.05 以增強初動力
        public static final double kDriveOutputScale = 1.0; // TODO Drive輸出縮放
        public static final boolean kEnableDrivePID = false; // TODO 是否啟用 Drive PID（啟用速度反饋控制）

    }

    /**
     * 飛輪驅動系統常數類別
     * 定義整個飛輪驅動系統的硬體配置和運動參數
     */
    public static final class DriveConstants {
        // === 機器人幾何參數 ===
        public static final double kTrackWidth = 0.22;  // TODO 左右輪距 (公尺)
        public static final double kWheelBase = 0.22;   // TODO 前後輪距 (公尺)

        // 飛輪驅動運動學計算，定義各輪子相對於機器人中心的位置
        // 順序：前左、前右、後左、後右
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // 前左輪位置   // TODO: 調整
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // 前右輪位置   // TODO: 調整
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // 後左輪位置   // TODO: 調整
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // 後右輪位置   // TODO: 調整

        // === 驅動馬達硬體名稱 ===
        public static final String kFrontLeftDriveMotorName = "BR";   // 前左驅動馬達
        public static final String kBackLeftDriveMotorName = "FR";    // 後左驅動馬達
        public static final String kFrontRightDriveMotorName = "BL";  // 前右驅動馬達
        public static final String kBackRightDriveMotorName = "FL";   // 後右驅動馬達

        // === 轉向馬達硬體名稱 ===
        public static final String kFrontLeftTurningMotorName = "BRTurn";   // 前左轉向馬達
        public static final String kBackLeftTurningMotorName = "FRTurn";    // 後左轉向馬達
        public static final String kFrontRightTurningMotorName = "BLTurn";  // 前右轉向馬達
        public static final String kBackRightTurningMotorName = "FLTurn";   // 後右轉向馬達

        // === 驅動編碼器方向設定 ===
        public static final boolean kFrontLeftDriveEncoderReversed = true;   // TODO 前左驅動編碼器反向
        public static final boolean kFrontRightDriveEncoderReversed = false;  // TODO 前右驅動編碼器反向

        public static final boolean kBackLeftDriveEncoderReversed = true;    // TODO 後左驅動編碼器反向
        public static final boolean kBackRightDriveEncoderReversed = false;   // TODO 後右驅動編碼器反向

        // === 轉向編碼器方向設定 ===
        public static final boolean kFrontLeftTurningEncoderReversed = true;   // TODO 前左轉向編碼器反向
        public static final boolean kFrontRightTurningEncoderReversed = true;  // TODO 前右轉向編碼器反向
        public static final boolean kBackLeftTurningEncoderReversed = true;    // TODO 後左轉向編碼器反向
        public static final boolean kBackRightTurningEncoderReversed = true;   // TODO 後右轉向編碼器反向

        // === 絕對編碼器硬體名稱 ===
        // 使用與 servo 相同的命名方式，便於硬體配置管理
        public static final String kFrontLeftAbsoluteEncoderName = "BREncoder";   // TODO 前左絕對編碼器
        public static final String kFrontRightAbsoluteEncoderName = "BLEncoder";  // TODO 前右絕對編碼器
        public static final String kBackLeftAbsoluteEncoderName = "FREncoder";    // TODO 後左絕對編碼器
        public static final String kBackRightAbsoluteEncoderName = "FLEncoder";   // TODO 後右絕對編碼器

        // === 絕對編碼器方向設定 ===
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;   // TODO 前左絕對編碼器反向
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;    // TODO 後左絕對編碼器反向
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;  // TODO 前右絕對編碼器反向
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;   // TODO 後右絕對編碼器反向

        // === 絕對編碼器偏移角度 (度數) ===
        // 用於校正輪子的初始方向，使用 SwerveModuleTuner 測量正確的偏移值
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 0.0;  // TODO 前左輪偏移角度
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = 0.0; // TODO 前右輪偏移角度
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = 0.0;   // TODO 後左輪偏移角度
        public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = 0.0;    // TODO 後右輪偏移角度

        // === 絕對編碼器偏移角度 (弧度) - 自動計算 ===
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(kFrontLeftDriveAbsoluteEncoderOffsetDeg);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(kBackLeftDriveAbsoluteEncoderOffsetDeg);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(kFrontRightDriveAbsoluteEncoderOffsetDeg);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(kBackRightDriveAbsoluteEncoderOffsetDeg);

        // === 機器人物理性能極限 ===
        // 理論值：5800 RPM × (1/8.8) × π × 0.058m ≈ 2.00 m/s（空載）
        // 實際負載下通常為理論值的 70~85%，建議執行 3. MaxSpeedAngularTest 量測後更新。
        public static final double kPhysicalMaxSpeedMetersPerSecond = 1.5;  // TODO 執行 _3_MaxSpeedAngularTest 量測後更新
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 10.879;    // TODO 機器人最大角速度 (弧度/秒)

        // === 手動控制性能限制 ===
        // 建議調整後進行測試以確保良好的操控手感
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;                    // TODO 手動控制最大線速度
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1;    // TODO 手動控制最大角速度
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;                                                 // 最大線性加速度
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;                                          // 最大角加速度

        // 各輪功率補償（1.0 = 不補償，> 1.0 = 加大功率補償阻力）
        public static final double kFrontLeftDrivePowerScale  = 1.00;  // TODO
        public static final double kFrontRightDrivePowerScale = 1.00;  // TODO
        public static final double kBackLeftDrivePowerScale   = 1.00;  // TODO
        public static final double kBackRightDrivePowerScale  = 1.00;  // TODO


        // === IMU 慣性測量單元配置 ===
        // 必須根據 Control Hub 的實際安裝方向進行設定
        public static final RevHubOrientationOnRobot.LogoFacingDirection kImuLogoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;        // TODO REV 標誌朝向
        public static final RevHubOrientationOnRobot.UsbFacingDirection kImuUsbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;    // TODO: USB 端口朝向

        // === GoBILDA Pinpoint 配置 ===
        // 設定為 true 使用 Pinpoint 進行定位，false 使用原本的 SwerveDriveOdometry + IMU
        public static final boolean USING_PINPOINT = true;
        public static final String kPinpointName = "pinpoint";  // Pin   point 在 hardwareMap 中的名稱

        // Pinpoint Odometry Pod 偏移量 (相對於機器人中心，單位：公釐)
        // X Pod 偏移：向左為正，向右為負
        // Y Pod 偏移：向前為正，向後為負
        public static final double kPinpointXPodOffsetMM = 0.0;   // TODO: 調整為實際值
        public static final double kPinpointYPodOffsetMM = -100.0;   // TODO: 調整為實際值

        // Pinpoint 編碼器方向
        // X: 前進為正，符合本專案座標定義
        // Y: Pinpoint 原廠預設左移為正，本專案慣例右移為正，故反向
        public static final boolean kPinpointXEncoderReversed = false;
        public static final boolean kPinpointYEncoderReversed = true;
    }

    public static final class AutoConstants {
        // === 軌跡速度限制 ===
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        // === X 軸 PID 控制器參數 (前後移動) ===
        public static final double kPXController = 0.0;   // P: 位置誤差增益
        public static final double kIXController = 0.0;   // I: 積分增益 (消除穩態誤差)
        public static final double kDXController = 0.0;   // D: 微分增益 (抑制震盪)

        // === Y 軸 PID 控制器參數 (左右移動) ===
        public static final double kPYController = 0.0;   // P: 位置誤差增益
        public static final double kIYController = 0.0;   // I: 積分增益
        public static final double kDYController = 0.0;   // D: 微分增益

        // === Theta 旋轉 PID 控制器參數 ===
        public static final double kPThetaController = 0.0;  // P: 角度誤差增益
        public static final double kIThetaController = 0.0;  // I: 積分增益
        public static final double kDThetaController = 0.0;  // D: 微分增益

        // === 容許誤差 ===
        public static final double kPositionToleranceMeters = 0.02;  // 位置容許誤差 (公尺)
        public static final double kAngleToleranceDegrees = 2.0;     // 角度容許誤差 (度)

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    /**
     * 操作者介面常數類別
     * 定義手把和輸入裝置的相關參數
     */
    public static final class OIConstants {
        public static final double kDeadband = 0.1;  // 手把搖桿死區範圍，避免微小抖動影響
    }
}
