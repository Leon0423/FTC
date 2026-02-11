# Absolute Encoder 讀數顯示

## 功能說明

現在您的 Swerve_Control OpMode 會在 telemetry 中顯示所有四個 Swerve 模組的 absolute encoder 讀數。

## Telemetry 顯示內容

當您運行 `Swerve_Control` OpMode 時，Driver Station 會顯示：

### 機器人狀態
- **Robot Heading**: 機器人當前朝向（度）
- **Robot Location**: 機器人在場地上的位置

### Absolute Encoder 讀數

每個模組顯示三種格式的讀數：

#### Front Left (FL)
- **FL Abs Encoder (V)**: 原始電壓值（Volts）
- **FL Abs Encoder (Rad)**: 角度（弧度，-π 到 π）
- **FL Abs Encoder (Deg)**: 角度（度數，-180° 到 180°）

#### Front Right (FR)
- **FR Abs Encoder (V)**: 原始電壓值
- **FR Abs Encoder (Rad)**: 角度（弧度）
- **FR Abs Encoder (Deg)**: 角度（度數）

#### Back Left (BL)
- **BL Abs Encoder (V)**: 原始電壓值
- **BL Abs Encoder (Rad)**: 角度（弧度）
- **BL Abs Encoder (Deg)**: 角度（度數）

#### Back Right (BR)
- **BR Abs Encoder (V)**: 原始電壓值
- **BR Abs Encoder (Rad)**: 角度（弧度）
- **BR Abs Encoder (Deg)**: 角度（度數）

## 新增的方法

### SwerveModule.java
```java
// 獲取原始電壓值
public double getAbsoluteEncoderVoltage()

// 獲取標準化的原始值 (0.0 到 1.0)
public double getAbsoluteEncoderRaw()

// 獲取計算後的角度（弧度）
public double getAbsoluteEncoderRad()
```

### SwerveSubsystem.java
```java
// 更新 telemetry，顯示所有 encoder 讀數
public void updateTelemetry(Telemetry telemetry)
```

## 使用方式

在您的 OpMode 主循環中，`swerveSubsystem.updateTelemetry(telemetry)` 會自動更新所有的 encoder 讀數。

這些讀數可以幫助您：
1. **調試轉向系統** - 確認每個輪子的實際角度
2. **校準 offset** - 找出正確的 encoder offset 值
3. **診斷硬體問題** - 檢查 encoder 是否正常工作
4. **優化控制** - 觀察輪子響應指令的速度

## 注意事項

- 電壓讀數範圍通常是 0V 到 3.3V（取決於您的 analog encoder）
- 角度會自動標準化到 [-π, π] 或 [-180°, 180°] 範圍
- 讀數已經考慮了 Constants.java 中設定的 offset 和 reversed 參數

