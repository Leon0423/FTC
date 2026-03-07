# 🚗 Swerve Drive 設定與測試指南

> 本文件提供新 Swerve 機器人的完整設定流程與測試方法

## 📋 目錄

1. [前置準備](#1-前置準備)
2. [硬體配置](#2-硬體配置)
3. [測試流程總覽](#3-測試流程總覽)
4. [Step 1: 編碼器偏移值校正](#step-1-編碼器偏移值校正)
5. [Step 2: 驅動馬達方向測試](#step-2-驅動馬達方向測試)
6. [Step 3: 最大速度測試](#step-3-最大速度測試)
7. [Step 4: 轉向 PID 調整](#step-4-轉向-pid-調整)
8. [Step 5: 驅動 PID 調整](#step-5-驅動-pid-調整)
9. [常見問題排解](#常見問題排解)
10. [Constants.java 參數說明](#constantsjava-參數說明)

---

## 1. 前置準備

### 1.1 軟體需求
- [ ] Android Studio 已安裝並配置完成
- [ ] FTCLib 已安裝
- [ ] FTC Dashboard 已安裝（用於即時參數調整）

### 1.2 硬體需求
- [ ] 4 個 DC 馬達（驅動馬達）
- [ ] 4 個 AXON Servo（轉向伺服機）：需要CRServo Mode
- [ ] 4 個絕對編碼器（AnalogInput）：通常整合於 AXON Servo
- [ ] 1 個 IMU：內建於 Control Hub
- [ ] REV Control Hub
---

## 2. 硬體配置

### 2.1 Driver Station 配置

在 FTC Driver Station 的「Configure Robot」中，設定以下硬體名稱：

#### 驅動馬達 (DcMotorEx)
| 名稱   | 說明     |
|------|--------|
| `FL` | 前左驅動馬達 |
| `FR` | 前右驅動馬達 |
| `BL` | 後左驅動馬達 |
| `BR` | 後右驅動馬達 |

#### 轉向伺服機 (CRServo)
| 名稱       | 說明      |
|----------|---------|
| `FLTurn` | 前左轉向伺服機 |
| `FRTurn` | 前右轉向伺服機 |
| `BLTurn` | 後左轉向伺服機 |
| `BRTurn` | 後右轉向伺服機 |

#### 絕對編碼器 (AnalogInput)
| 名稱          | 說明      |
|-------------|---------|
| `FLEncoder` | 前左絕對編碼器 |
| `FREncoder` | 前右絕對編碼器 |
| `BLEncoder` | 後左絕對編碼器 |
| `BREncoder` | 後右絕對編碼器 |

### 2.2 修改硬體名稱（選用）

如果您的硬體名稱不同，請修改 `Constants.java`：

```java
// Drive Motors
public static final String kFrontLeftDriveMotorName = "FL";
public static final String kBackLeftDriveMotorName = "BL"; 
public static final String kFrontRightDriveMotorName = "FR";
public static final String kBackRightDriveMotorName = "BR";

// Turning Servos
public static final String kFrontLeftTurningMotorName = "FLTurn";
public static final String kBackLeftTurningMotorName = "BLTurn";
public static final String kFrontRightTurningMotorName = "FRTurn"; 
public static final String kBackRightTurningMotorName = "BRTurn";

// Absolute Encoders
public static final String kFrontLeftAbsoluteEncoderName = "FLEncoder";
public static final String kFrontRightAbsoluteEncoderName = "FREncoder";
public static final String kBackLeftAbsoluteEncoderName = "BLEncoder";
public static final String kBackRightAbsoluteEncoderName = "BREncoder";
```

---

## 3. 測試流程總覽

```
┌─────────────────────────────────────────────────────────────────┐
│                    Swerve 設定流程圖                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Step 1: Encoder Offset Reader                                  │
│  ─────────────────────────────                                  │
│  • 讀取並校正絕對編碼器偏移值                                       │
│  • 將數值填入 Constants.java                                      │
│                    ↓                                            │
│  Step 2: Drive Motor Direction Tester                           │
│  ─────────────────────────────────────                          │
│  • 確認輪子對齊是否正確                                            │
│  • 測試驅動馬達方向                                              │
│  • 修正反向設定                                                  │
│                    ↓                                            │
│  Step 3: Max Speed Angular Test                                 │
│  ──────────────────────────────                                 │
│  • 測量最大直線速度                                              │
│  • 測量最大角速度                                                │
│  • 更新 Constants.java 的物理極限值                              │
│                    ↓                                            │
│  Step 4: Turning PID Tuner                                      │
│  ─────────────────────────                                      │
│  • 調整轉向 PID 參數                                             │
│  • 確保輪子能快速準確地轉到目標角度                              │
│                    ↓                                            │
│  Step 5: Drive PID Tuner (選用)                                 │
│  ────────────────────────────                                   │
│  • 調整驅動速度 PID 參數                                         │
│  • 提升速度控制精準度                                            │
│                    ↓                                            │
│  ✅ 完成！可以開始使用 Swerve_Control                            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Step 1: 編碼器偏移值校正

### 📍 目的
校正每個 Swerve 模組的絕對編碼器偏移值，使輪子的「0 度」對應車頭方向。
```
     車頭↑
┌─────────────┐
|   |    |    |
|   |    |    | 
└─────────────┘
```

### 📱 OpMode 名稱
`1. Encoder Offset Reader`（Tuning 群組）

### 📝 操作步驟

1. **啟動 OpMode**
   - 在 Driver Station 選擇 `1. Encoder Offset Reader`
   - 按下 **Init & Start**

2. **對齊輪子**
   - 手動將**所有 4 個輪子**轉到**車頭方向**
   - 確保輪子完全平行於機器人的縱軸

3. **記錄偏移值**
   - 觀察 Telemetry 上顯示的「Raw Angle」數值
   - 記錄 4 個輪子的角度：
     - `FL OffsetDeg: XXX.X`
     - `FR OffsetDeg: XXX.X`
     - `BL OffsetDeg: XXX.X`
     - `BR OffsetDeg: XXX.X`

4. **更新 Constants.java**
   ```java
   // 絕對編碼器偏移角度 (度數)
   public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 116.6;  // 填入 FL 的值
   public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = 186.4; // 填入 FR 的值
   public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = 188.7;   // 填入 BL 的值
   public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = 4.7;    // 填入 BR 的值
   ```

5. **重新部署程式**
   - 將更新後的程式上傳到 Control Hub

### ✅ 驗證方式
- 重新執行此 OpMode
- Init 後觀察「Cur」（當前角度）應該接近 0°
- 如果偏差超過 5°，請重新校正

---

## Step 2: 驅動馬達方向測試

### 📍 目的
確認驅動馬達的正反轉設定正確，使所有輪子在「前進」指令下都往同一方向轉動。

### 📱 OpMode 名稱
`2. Drive Motor Direction Tester`（Tuning 群組）

### 📝 前置條件
⚠️ **必須先完成 Step 1**，確保編碼器偏移值已正確設定。

### 📝 操作步驟

1. **啟動 OpMode**
   - 選擇 `2. Drive Motor Direction Tester`
   - 按下 **Init**

2. **觀察輪子對齊**
   - Init 階段會自動將所有輪子轉到 0°（車頭方向）
   - 確認 Telemetry 顯示 `★ 所有輪子已對齊！可以按 Start ★`
   - 如果輪子無法對齊，請回到 Step 1 重新校正

3. **按下 Start 開始測試**

4. **使用手把測試**

   | 按鈕 | 功能 | 預期結果 |
   |------|------|----------|
   | A | 全部前進 | 所有輪子往車頭方向滾動 |
   | B | 全部後退 | 所有輪子往車尾方向滾動 |
   | X | 左側前進 | FL 和 BL 往前滾動 |
   | Y | 右側前進 | FR 和 BR 往前滾動 |
   | ↑ | FL 單獨 | 只有 FL 轉動 |
   | → | FR 單獨 | 只有 FR 轉動 |
   | ← | BL 單獨 | 只有 BL 轉動 |
   | ↓ | BR 單獨 | 只有 BR 轉動 |

5. **觀察 Encoder 數值**
   - 按 A（前進）時，所有 Encoder 數值應該**增加**
   - 如果某個馬達的數值減少，代表該馬達需要反轉

6. **修正反向設定**
   如果某個馬達方向錯誤，修改 `Constants.java`：
   ```java
   // 將 false 改為 true，或 true 改為 false
   public static final boolean kFrontLeftDriveEncoderReversed = false;
   public static final boolean kBackLeftDriveEncoderReversed = false;
   public static final boolean kFrontRightDriveEncoderReversed = false;
   public static final boolean kBackRightDriveEncoderReversed = false;
   ```

### ✅ 驗證方式
- 按 A 時所有 Encoder 數值增加
- 按 B 時所有 Encoder 數值減少
- 輪子實際滾動方向與預期相符

---

## Step 3: 最大速度測試

### 📍 目的
測量機器人的實際最大直線速度和最大角速度，用於優化控制參數。

### 📱 OpMode 名稱
`3. MaxSpeedAngularTest`（Tuning 群組）

### 📝 前置條件
⚠️ **必須先完成 Step 1 和 Step 2**

### ⚠️ 安全警告
- 此測試需要機器人**在地面上運行**
- 請確保有**足夠空間**（至少 3m x 3m）
- 測試時**遠離機器人**

### 📝 操作步驟

1. **啟動 OpMode**
   - 選擇 `3. MaxSpeedAngularTest`
   - 按下 **Init**

2. **選擇測試模式**

   | 按鈕 | 功能 |
   |------|------|
   | A | 直線速度測試 |
   | B | 角速度測試 |
   | DPAD UP | 增加功率 (+10%) |
   | DPAD DOWN | 減少功率 (-10%) |

3. **直線速度測試**
   - 按 A 選擇直線模式
   - 按 Start 開始測試
   - 機器人會直線前進約 3 秒
   - 記錄顯示的 `kPhysicalMaxSpeedMetersPerSecond` 值

4. **角速度測試**
   - 重新 Init
   - 按 B 選擇角速度模式
   - 按 Start 開始測試
   - 機器人會原地旋轉約 3 秒
   - 記錄顯示的 `kPhysicalMaxAngularSpeedRadiansPerSecond` 值

5. **更新 Constants.java**
   ```java
   // 機器人物理性能極限
   public static final double kPhysicalMaxSpeedMetersPerSecond = X.XXX;
   public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = X.XXX;
   ```

### 💡 建議
- 第一次測試建議使用 50% 功率，確認安全後再將功率改成100%測試
- 多次測試取平均值以獲得更準確的結果

---

## Step 4: 轉向 PID 調整

### 📍 目的
調整轉向 PID 參數，使輪子能快速、準確、穩定地轉到目標角度。

### 📱 OpMode 名稱
`4. Turning PID Tuner`（Tuning folder）

### 🔧 工具
- FTC Dashboard（http://192.168.43.1:8080/dash）

### 📝 操作步驟

1. **連接 FTC Dashboard**
   - 確保電腦與 Control Hub 在同一網路
   - 開啟瀏覽器訪問 `http://192.168.43.1:8080/dash`

2. **啟動 OpMode**
   - 選擇 `4. Turning PID Tuner`
   - 按下 **Init**，然後按 **Start**

3. **觀察 Dashboard**
   - **Config 頁籤**：調整參數
   - **Graph 頁籤**：觀察響應曲線
     - `target`：目標角度（紅色）
     - `FL`, `FR`, `BL`, `BR`：實際角度

4. **調整 PID 參數**

   | 參數 | 說明 | 調整方式 |
   |------|------|----------|
   | `_1a_turningP` | 比例增益 | 從 0.3 開始，逐步增加 |
   | `_1b_turningI` | 積分增益 | 通常設為 0 |
   | `_1c_turningD` | 微分增益 | 如果震盪，小幅增加 |
   | `_1d_outputScale` | 輸出縮放 | 通常設為 1.0 |
   | `_1e_deadbandDeg` | 死區角度 | 2-5 度 |

5. **PID 調整技巧**

   ```
   調整順序：
   1. 設定 I = 0, D = 0
   2. 增加 P 直到響應快速但稍微過衝 (overshoot)
   3. 增加 D 來抑制過衝
   4. 如果有穩態誤差，小幅增加 I
   ```

### ✅ 良好的 PID 表現
- 響應時間 < 0.5 秒
- 過衝量 < 10%
- 穩態誤差 < 3°
- 無持續震盪

### 📊 Dashboard 圖表判讀

```
良好的響應：
    ^
    |    ___________
    |   /
    |  /
    | /
    |/
    +---------------->
                time

過衝 (需增加 D 或減少 P)：
    ^
    |  /\
    | /  \___________
    |/
    +---------------->

響應過慢 (需增加 P)：
    ^
    |        __________
    |       /
    |    __/
    |___/
    +---------------->

震盪 (需減少 P 或增加 D)：
    ^
    |  /\  /\  /\
    | /  \/  \/  \____
    |/
    +---------------->
```

---

## Step 5: 驅動 PID 調整

### 📍 目的
調整驅動速度 PID 參數，提升速度控制精準度（選用）。

### 📱 OpMode 名稱
`5. Drive PID Tuner`（Tuning 群組）

### ⚠️ 注意
- Drive PID 預設為**關閉**狀態
- 只有需要精準速度控制時才需要啟用
- 此測試需要機器人**在地面上運行**

### 📝 操作步驟

1. **啟動 OpMode**
   - 選擇 `5. Drive PID Tuner`
   - 按下 **Init**，然後按 **Start**

2. **在 Dashboard 調整參數**
   - 位置：`Config > TuningConfig`
   - 先啟用：`_4a_enableDrivePID = true`

3. **調整 PID + F 參數**

   | 參數 | 說明 | 建議值 |
   |------|------|--------|
   | `_3a_driveP` | 比例增益 | 0.05 - 0.2 |
   | `_3b_driveI` | 積分增益 | 0 |
   | `_3c_driveD` | 微分增益 | 0 |
   | `_3d_driveF` | 前饋增益 | 0.8 - 1.0 |

4. **調整順序**
   ```
   1. 設定 P, I, D = 0
   2. 調整 F 使馬達在目標速度下大致達到
   3. 增加 P 來修正剩餘誤差
   4. 如有穩態誤差，小心增加 I
   ```

5. **控制按鍵**
   | 按鈕 | 功能 |
   |------|------|
   | X | 暫停測試，手動控制 |
   | Y | 繼續自動測試 |

### ✅ 驗證方式
- 目標速度與實際速度差距 < 10%
- 無明顯震盪

---

## 常見問題排解

### Q1: 輪子無法對齊到 0 度
**可能原因：**
- 編碼器偏移值設定錯誤
- 轉向伺服機方向設定錯誤

**解決方法：**
1. 重新執行 Step 1 校正偏移值
2. 檢查 `kXXXTurningEncoderReversed` 設定

### Q2: 機器人向前指令但往後移動
**可能原因：**
- 驅動馬達方向設定錯誤

**解決方法：**
1. 執行 Step 2 測試馬達方向
2. 修改 `kXXXDriveEncoderReversed` 設定

### Q3: 輪子轉動時抖動或震盪
**可能原因：**
- Turning PID 的 P 值過高
- D 值不足

**解決方法：**
1. 降低 `kPTurning` 值
2. 增加 `kDTurning` 值
3. 增加 `kTurningDeadbandDeg` 死區

### Q4: 輪子轉動太慢
**可能原因：**
- Turning PID 的 P 值過低
- 輸出縮放過低

**解決方法：**
1. 增加 `kPTurning` 值
2. 確認 `kTurningOutputScale` = 1.0

### Q5: 機器人漂移或無法直線行走
**可能原因：**
- IMU 配置方向錯誤
- 輪子對齊不準確

**解決方法：**
1. 檢查 IMU 方向設定：
   ```java
   public static final RevHubOrientationOnRobot.LogoFacingDirection kImuLogoFacingDirection
   public static final RevHubOrientationOnRobot.UsbFacingDirection kImuUsbFacingDirection
   ```
2. 重新校正編碼器偏移值

### Q6: 硬體找不到錯誤
**錯誤訊息範例：**
```
FL Servo: FLTurn 找不到!
```

**解決方法：**
1. 確認 Driver Station 的硬體配置名稱正確
2. 確認硬體已正確連接
3. 重啟 Control Hub

---

## Constants.java 參數說明

### 機械參數
```java
public static final double kWheelDiameterMeters = 0.08;  // 輪子直徑 (公尺)
public static final double kTrackWidth = 0.33;           // 左右輪距 (公尺)
public static final double kWheelBase = 0.33;            // 前後輪距 (公尺)
```

### 轉向 PID 參數
```java
public static final double kPTurning = 0.43;         // P 係數
public static final double kITurning = 0.0;          // I 係數
public static final double kDTurning = 0.0;          // D 係數
public static final double kTurningOutputScale = 1.0; // 輸出縮放
public static final double kTurningDeadbandDeg = 3.0; // 死區角度 (度)
```

### 驅動 PID 參數
```java
public static final double kPDrive = 0.1;            // P 係數
public static final double kIDrive = 0.0;            // I 係數
public static final double kDDrive = 0.0;            // D 係數
public static final double kFDrive = 0.0;            // F 前饋係數
public static final double kDriveOutputScale = 1.0;  // 輸出縮放
public static final boolean kEnableDrivePID = false; // 是否啟用 Drive PID
```

### 物理極限
```java
public static final double kPhysicalMaxSpeedMetersPerSecond = 11.6;          // 最大直線速度
public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4*PI;  // 最大角速度
```

### 手動控制限制
```java
public static final double kTeleDriveMaxSpeedMetersPerSecond = ...;          // 手動最大直線速度
public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = ...;  // 手動最大角速度
public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;      // 最大加速度
```

### IMU 方向配置
```java
public static final RevHubOrientationOnRobot.LogoFacingDirection kImuLogoFacingDirection = 
    RevHubOrientationOnRobot.LogoFacingDirection.UP;
public static final RevHubOrientationOnRobot.UsbFacingDirection kImuUsbFacingDirection = 
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
```

---

## 📞 完成設定檢查清單

在開始比賽前，請確認：

- [ ] Step 1: 編碼器偏移值已校正，所有輪子 Init 時對齊 0°
- [ ] Step 2: 驅動馬達方向正確，前進時所有輪子同向轉動
- [ ] Step 3: 已測量並更新最大速度參數
- [ ] Step 4: 轉向 PID 已調整，輪子能快速穩定轉到目標角度
- [ ] Step 5: (選用) 驅動 PID 已調整
- [ ] IMU 方向配置正確
- [ ] 機器人能正常直線行走和旋轉
- [ ] 手動控制（Swerve_Control）運作正常

---

## 📌 快速參考

### OpMode 執行順序
```
1. Encoder Offset Reader    → 校正偏移值
2. Drive Motor Direction    → 測試馬達方向
3. MaxSpeedAngularTest      → 測量最大速度
4. Turning PID Tuner        → 調整轉向 PID
5. Drive PID Tuner          → 調整驅動 PID (選用)
```

### FTC Dashboard URL
```
http://192.168.43.1:8080/dash
```

### 主要控制 OpMode
```
Swerve_Control    → 比賽用主程式
```

---

*最後更新：2026年2月*
*版本：1.0*

