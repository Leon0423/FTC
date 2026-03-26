# FTC Swerve Drive 完整使用說明

**FTC 2025–2026 賽季 ｜ Java / FTCLib ｜ REV Control Hub**

---

## 目錄

1. [硬體架構](#1-硬體架構)
2. [首次安裝與硬體配置](#2-首次安裝與硬體配置)
3. [Constants.java 參數說明](#3-constantsjava-參數說明)
4. [Tuning 程式流程（必照順序執行）](#4-tuning-程式流程必照順序執行)
5. [正式操作：Swerve_Control](#5-正式操作-swerve_control)
6. [自動 PID 定點校正：Auto PID Tuner](#6-自動-pid-定點校正auto-pid-tuner)
7. [FTC Dashboard 使用方式](#7-ftc-dashboard-使用方式)
8. [TuningConfig 即時調參說明](#8-tuningconfig-即時調參說明)
9. [角度追蹤系統原理](#9-角度追蹤系統原理)
10. [常見問題排查](#10-常見問題排查)
11. [待辦與已知限制](#11-待辦與已知限制)

---

## 1. 硬體架構

| 硬體           | 規格 / 說明                                      |
|--------------|----------------------------------------------|
| **DC Motor** | goBILDA DcMotorEx（28 ticks/rev）              |
| **轉向 Servo** | AXON CRServo（CRServo Mode）                   |
| **轉向編碼器**    | AXON 內建絕對編碼器（AnalogInput，0 ~ 3.3 V）          |
| **齒輪比（轉向）**  | 0.4（Servo 轉 2.5 圈 = 輪子轉 1 圈）                 |
| **最大線速度**    | 0.764 m/s（實測，100% 功率）                        |
| **最大角速度**    | 10.879 rad/s（實測，100% 功率）                     |
| **主控**       | REV Control Hub                              |
| **IMU**      | Control Hub 內建 BNO055（可切換為 GoBILDA Pinpoint） |

### 硬體命名表（hardwareMap 名稱）

| 類型 | 前左 FL | 前右 FR | 後左 BL | 後右 BR |
|------|---------|---------|---------|---------|
| 驅動馬達 (DcMotorEx) | `BR` | `BL` | `FR` | `FL` |
| 轉向伺服 (CRServo) | `BRTurn` | `BLTurn` | `FRTurn` | `FLTurn` |
| 絕對編碼器 (AnalogInput) | `BREncoder` | `BLEncoder` | `FREncoder` | `FLEncoder` |
| IMU | `imu` | — | — | — |

> **注意：** hardwareMap 命名將會對應車頭位置，請修改 `Constants.java` 中對應的字串常數。

---

## 2. 首次安裝與硬體配置

### 2.1 在 Driver Station 設定硬體

1. 開啟 **FTC Driver Station App**
2. 進入 **Configure Robot**
3. 新增或編輯設定檔，依照上表加入所有硬體裝置並指定正確的 Port
4. 確認 IMU 名稱為 `imu`
5. 儲存設定

### 2.2 安裝 FTC Dashboard（選用但強烈建議）

1. 在 `build.dependencies.gradle` 加入 FTC Dashboard (目前這份檔案已安裝)
2. 連線後以瀏覽器開啟 `http://192.168.43.1:8080/dash` (後續需調整PID時再打開
)

### 2.3 專案檔案結構

```
teamcode/
├── Constants.java               ← 所有常數集中在這裡
├── Swerve_Control.java          ← 正式 TeleOp
├── AutoPIDTuner.java            ← 自動路徑 PID 調整程式
├── commands/
│   └── SwerveJoystickCmd.java   ← 搖桿指令
├── subsystems/
│   ├── SwerveModule.java        ← 單輪邏輯（轉向追蹤、PID）
│   └── SwerveSubsystem.java     ← 四輪整合、里程計、IMU
└── Tuning/
    ├── TuningConfig.java
    ├── SlewRateLimiter.java
    ├── Units.java
    ├── _1a_EncoderOffsetReader.java
    ├── _1b_TurningEncoderReverse.java
    ├── _1c_AutoSetOffset.java
    ├── _1c_ClearSwervePrefs.java
    ├── _2_DriveMotorDirectionTester.java
    ├── _3_MaxSpeedAngularTest.java
    ├── _4_TurningPIDTuner.java
    └── _5_DrivePIDTuner.java
```

---

## 3. Constants.java 參數說明

### ModuleConstants（單輪參數）

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `kWheelDiameterMeters` | 輪子直徑（公尺） | `0.058` |
| `kDriveMotorGearRatio` | 驅動馬達齒輪比 | `0.113636` |
| `kServoTurnsPerWheelTurn` | Servo 轉幾圈對應輪子轉 1 圈 | `2.5` |
| `kPTurning` | 轉向 PID — P 係數 | `0.5` |
| `kITurning` | 轉向 PID — I 係數 | `0.02` |
| `kDTurning` | 轉向 PID — D 係數 | `0.00` |
| `kTurningDeadbandDeg` | 轉向死區（度） | `0.0` |
| `kPDrive / kIDrive / kDDrive / kFDrive` | 驅動 PID 各係數 | 全 `0.0` |
| `kEnableDrivePID` | 是否啟用驅動 PID | `false` |

### DriveConstants（整車參數）

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `kTrackWidth` | 左右輪距（公尺） | `0.22` |
| `kWheelBase` | 前後輪距（公尺） | `0.22` |
| `kPhysicalMaxSpeedMetersPerSecond` | 實測最大線速度 | `0.764` |
| `kPhysicalMaxAngularSpeedRadiansPerSecond` | 實測最大角速度 | `10.879` |
| `kFrontLeftDrivePowerScale` 等 | 各輪功率補償 | 全 `1.00` |
| `kImuLogoFacingDirection` | Control Hub Logo 朝向 | `UP` |
| `kImuUsbFacingDirection` | Control Hub USB 朝向 | `BACKWARD` |
| `USING_PINPOINT` | 切換為 GoBILDA Pinpoint 定位 | `false` |

### AutoConstants（自動 PID 參數）

所有 PID 初始值皆為 `0.0`，需透過 AutoPIDTuner 調整後填入。

---

## 4. Tuning 程式流程（必照順序執行）

> **重要：** 第一次使用或換裝零件後，務必照 **1a → 1b → 1c → 2 → 3 → 4 → 5** 的順序執行。

---

### Step 1a：Encoder Offset Reader

**目的：** 監看絕對編碼器三層數值，確認硬體接線正常。

**操作：** 開啟程式（不需按 Start），手動轉動輪子觀察數值。

| 層次 | 說明 |
|------|------|
| **第 1 層 Raw** | 編碼器電壓換算成 0~360°，無任何處理 |
| **第 2 層 Geared** | Raw × 齒輪比（0.4），輪子實際轉角（未扣 Offset） |
| **第 3 層 Final** | 扣掉 Offset 後，SwerveModule 實際使用的值 |

**合格：** 轉動時數值連續變化，無突然跳動。

---

### Step 1b：Turning Encoder Reverse

**目的：** 確認 CRServo 正功率時編碼器角度是否增加。

**操作：** 按 Start → 按 A 發送 Pulse（300ms），觀察角度變化。

| 結果 | 設定 |
|------|------|
| 角度增加 | `kXXXTurningEncoderReversed = false` |
| 角度減少 | `kXXXTurningEncoderReversed = true` |

按鍵：`A` 發送 Pulse，`LB/RB` 調整功率 ±10%

---

### Step 1c：Auto Set Offset

**目的：** 手動對齊輪子朝前後，一鍵寫入 Offset。

**操作：**
1. 手動將四個輪子全部轉到朝向正前方
2. 按 Start → 觀察第 3 層 Final 是否接近 0°
3. 全部 < ±5° 後按 **A** 寫入

> 按 A 同時清除累積角追蹤記憶。此程式會覆蓋既有 Offset。

---

### Step 1c：Clear Swerve Prefs（重置用）

**目的：** 清除 Control Hub 所有持久化角度與 Offset 記錄。

**使用時機：** 角度追蹤嚴重跑掉、換裝零件、記憶被汙染。

**操作：** 按 Start 即立即清除，再重新執行 Auto Set Offset。

---

### Step 2：Drive Motor Direction Tester

**目的：** 確認四顆驅動馬達前進時 Encoder 數值增加。

**Init 階段：** 程式自動對齊四輪到 0°，等待「✅ 全部對齊」後按 Start。

**測試：**

| Encoder 變化 | 結論 |
|-------------|------|
| 數值增加（↑） | `kXXXDriveEncoderReversed = false` |
| 數值減少（↓） | `kXXXDriveEncoderReversed = true` |

| 按鍵 | 功能 |
|------|------|
| `A` | 全部前進 |
| `B` | 全部後退 |
| `DPAD ↑ → ← ↓` | FL / FR / BL / BR 單獨 |
| `LB / RB` | 調整功率 ±10% |
| `Y` | 套用結論到本次執行期 |

---

### Step 3：Max Speed & Angular Test

**目的：** 實測最大線速度與角速度，填入 Constants.java。

**操作：** 以 100% 功率分別測試直走與旋轉，記錄穩定數值。

```java
public static final double kPhysicalMaxSpeedMetersPerSecond = [實測值];
public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = [實測值];
```

> **重要：** 兩個值必須在**相同功率（100%）** 下測量，否則 Field-Oriented 的角速度比例失準，導致邊走邊轉時向對角線偏移。

---

### Step 4：Turning PID Tuner

**目的：** 調整轉向 CRServo PID，使輪角快速穩定跟上正弦波目標。

**操作：**
1. 連接 Dashboard，按 Start
2. 在 Graph 觀察 `target_deg` 與各輪角度曲線
3. 在 Config → `_4_TurningPIDTuner` 即時調整

**建議順序：** 先調 P（0.3 起）→ 加 D 減震盪 → 有穩態誤差才加 I

| 參數 | 說明 |
|------|------|
| `_1a/b/c_turningP/I/D` | PID 係數 |
| `_1d_outputScale` | 輸出縮放（通常 1.0） |
| `_1e_deadbandDeg` | 死區（建議 0） |
| `_2a_testPeriodSec` | 正弦波週期（秒） |
| `_2b_testAmplitudeDeg` | 正弦波振幅（度） |

**完成後：** 將值填入 `Constants.java` 的 `kPTurning / kITurning / kDTurning`。

---

### Step 5：Drive PID Tuner

**目的：** 調整驅動馬達速度控制，使實際速度跟上方波目標。

**Init 選擇測試模組：** `A` = 全部，`DPAD ↑→←↓` = FL/FR/BL/BR 單獨。

**建議流程：**
```
1. fOnlyMode = true，調 manualF 讓速度大致正確
2. 需精準控制時關閉 fOnlyMode，依序加 F → P → D → I
3. 始終震盪則維持 F-Only，保持 kEnableDrivePID = false
```

| 按鍵 | 功能 |
|------|------|
| `X` | 暫停 / 繼續 |
| `LB / RB` | 調整測試最大速度 ±0.05 m/s |

---

## 5. 正式操作：Swerve_Control

### 搖桿配置

| 搖桿 / 按鍵 | 功能 |
|------------|------|
| **左搖桿 上/下** | 前進 / 後退 |
| **左搖桿 左/右** | 左移 / 右移（側移） |
| **右搖桿 左/右** | 逆時針 / 順時針旋轉 |
| **LB** | 切換 Field-Oriented ↔ Robot-Oriented |
| **A** | 重置 Heading 並重建角度追蹤 |
| **Y** | 切換 Lock Mode（閒置時 X 型鎖輪） |
| **DPAD ↑ / ↓** | 直走速度倍率 ±10%（範圍 10% ~ 100%） |
| **DPAD → / ←** | 轉動速度倍率 ±10% |

### 控制模式

**Field-Oriented（預設）：** 以場地為基準，推上搖桿永遠往場地正前方，適合比賽操作。

**Robot-Oriented：** 以機器人車頭為基準，適合調試。

**Lock Mode：** 閒置時輪子擺成 X 型（±45°），增加防滑阻力。

### Dashboard 監看資訊

| 欄位 | 說明 |
|------|------|
| `Mode` | 控制模式 |
| `Heading` | 當前航向（度） |
| `Position X/Y` | 里程計估計位置（公尺） |
| `Drive RPM` | 各輪 RPM |
| `Turning Angle` | 各輪轉向角（度） |
| `Accum Wheel Angle` | 各輪累積角（驗證追蹤連續性） |
| `Speed / Turning` | 目前速度 / 轉動倍率 |

### 結束程式

按 Stop 後：程式執行 600ms X 型鎖輪收斂 → 停止驅動馬達（保留最後轉向輸出）。

---

## 6. 自動 PID 定點校正：Auto PID Tuner

**目的：** 調整自動模式 X / Y / Theta 位置 PID，使機器人被推離後能自動回正。

**操作：**
1. 連接 Dashboard，選擇測試模式

| 模式值 | 說明 |
|-------|------|
| `0` | 只測 X 軸（前後） |
| `1` | 只測 Y 軸（左右） |
| `2` | 只測 Theta（旋轉） |
| `3` | 三軸全部 |

2. 按 Start，用手推機器人觀察回正效果
3. 在 Dashboard 即時調整直到滿意
4. 將最終值填入 `Constants.java` 的 `AutoConstants`

---

## 7. FTC Dashboard 使用方式

**連線：** 手機/筆電連到 Control Hub Wi-Fi，瀏覽器開啟 `http://192.168.43.1:8080/dash`

| 頁籤 | 說明 |
|-----|------|
| **Graph** | 即時折線圖（`packet.put()` 的數值） |
| **Config** | 即時調整 `@Config` 標注的 `public static` 參數 |
| **Field** | 場地俯視圖（藍色 = 當前，綠色 = 目標） |
| **Telemetry** | Driver Station telemetry 內容 |

---

## 8. TuningConfig 即時調參說明

Dashboard → Config → `TuningConfig` 可即時修改（無需重新部署）：

| 參數 | 說明 |
|------|------|
| `_1a/b/c_turningP/I/D` | 轉向 PID 係數 |
| `_1d_turningOutputScale` | 轉向輸出縮放 |
| `_2a_deadbandDeg` | 轉向死區（度） |
| `_3a/b/c/d_driveP/I/D/F` | 驅動 PID 各係數 |
| `_3e_driveOutputScale` | 驅動輸出縮放 |
| `_4a_enableDrivePID` | 啟用 / 停用驅動 PID |
| `_5a_teleDriveMaxAccel` | TeleOp 最大線性加速度 |
| `_5b_teleDriveMaxAngularAccel` | TeleOp 最大角加速度 |

> Dashboard 調整只在本次執行期有效，滿意後手動更新 `Constants.java`。

---

## 9. 角度追蹤系統原理

### 為什麼不能直接用絕對編碼器？

Servo 每轉一圈編碼器歸零，但輪子每轉一圈需要 Servo 轉 2.5 圈。同一個編碼器讀數對應**三個不同輪子位置**（0°、144°、288°），無法唯一定位。

### 解決方案：Delta 累積追蹤

```
每個迴圈：
  delta = 當前 Servo 角度 - 上次 Servo 角度（處理跨 0 邊界）
  accumulatedAngle += delta × 齒輪比（0.4）
```

### 四大保護機制

| 機制 | 說明 |
|------|------|
| **毫秒快取** | 同一毫秒多次呼叫直接回傳快取值，防止 delta 重複累積 |
| **持久化儲存** | 關機時將累積角存入 SharedPreferences，下次開機續接 |
| **靜止校正** | 靜止 > 500ms 且差距 < 10° 時一次性修正累積誤差 |
| **啟動合理性檢查** | 差 < 10° 信任；10~30° 加權混合；> 30° 退回絕對編碼器 |

### IMU 航向追蹤

使用軟體 Offset 解決 `imu.resetYaw()` 非同步問題：

```java
// zeroHeading() 記錄基準值
headingOffset = raw;
// getHeading() 回傳差值
return raw - headingOffset;
```

另有 20ms 快取（50Hz）防止頻繁 I2C 讀取。

---

## 10. 常見問題排查

### 啟動後輪子跑到奇怪方向

執行 `1c. ClearSwervePrefs` 清除記憶，再重新執行 `1c. Auto Set Offset`。

---

### 直走時機器人偏向某一側

**原因 A：** 某顆馬達轉速不足 → 調整 `kXXXDrivePowerScale`（> 1.0 加大補償）

**原因 B：** Physical Max 常數測量功率不一致 → 用相同功率（100%）重測兩個值

---

### 轉 90° 後前後顛倒

確認 `SwerveSubsystem.java` 中有負號：
```java
public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-getHeading());  // 必須有負號
}
```

---

### speedMultiplier 調整後車速沒有變化

確認 Lambda 直接引用成員變數：
```java
() -> -driverGamepad.getLeftY() * speedMultiplier
```
Lambda 自動捕捉外部成員變數，不需重建 `joystickCmd`。

---

### 長時間運行後輪角漂移

靜止校正機制會自動修正小誤差；根本解法：啟用 GoBILDA Pinpoint（`USING_PINPOINT = true`）。

---

### Tuning 程式污染角度記憶

執行 `1c. ClearSwervePrefs`，或重新執行 `1c. Auto Set Offset`。

---

### 機器人閒置時輪子持續微動

降低 `_1a_turningP`（在 TuningConfig 即時調整）；確認 `STILL_THRESHOLD_MS = 500` 未被設太短。

---

## 11. 待辦與已知限制

| 問題 | 當前狀態 | 計畫 |
|------|---------|------|
| FL 馬達比其他慢約 20% | 用 `kFrontLeftDrivePowerScale` 暫時補償 | 比賽後拆機確認機構問題 |
| IMU 長時間漂移 | 20ms 快取降低影響 | 啟用 GoBILDA Pinpoint |
| 2.5 圈絕對定位限制 | Delta 追蹤 + 持久化儲存 | 換多圈絕對編碼器 |
| 電壓降幅偏大 | 未解決 | 確認電池老化或接頭接觸不良 |
| Auto PID 未調整 | Constants 全設為 0.0 | 使用 AutoPIDTuner 調整後填入 |

---

*文件版本：2025–2026 賽季 ｜ 最後更新：2026-03*
