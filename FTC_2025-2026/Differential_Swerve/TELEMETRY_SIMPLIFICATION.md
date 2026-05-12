# Swerve_Control 遙測簡化說明

## 修改目標
簡化 Swerve_Control 和 SwerveSubsystem 的遙測輸出，只顯示必要資訊在 FTC Dashboard 的 Graph 和 Telemetry。

## 修改內容

### 1. Swerve_Control.java

#### 移除的內容：
- **SlewRateLimiter 視覺化變數和初始化**
  - 移除了 `xLimiterViz`, `yLimiterViz`, `turnLimiterViz`
  - 移除了所有 `rawXInput`, `limitedXInput` 等視覺化變數
  - 移除了在初始化時創建 SlewRateLimiter 的代碼

- **詳細的搖桿輸入遙測**
  - 移除了 X、Y、Turn 軸的 Raw Input / Limited Output / Difference 顯示
  - 移除了 `=== SlewRateLimiter Visualization ===` 區塊

#### 保留的內容：
- **必要的遙測資訊**：
  - `Mode`: Field-Oriented 或 Robot-Oriented
  - `Heading`: 機器人當前航向角度
  - `Position`: 機器人當前位置 (X, Y)
  - `Target`: 目標位置 (X, Y, Heading)
  - `Status`: 運行狀態

### 2. SwerveSubsystem.java

#### `updateTelemetry()` 方法簡化：
移除的內容：
- Turning PID 參數顯示 (P/I/D 值)
- 四個模組的詳細轉向遙測 (target, current, error, output)
- 絕對編碼器的詳細讀數：
  - 電壓值 (V)
  - 弧度值 (Rad)
  - 角度值 (Deg)

保留的內容：
- Robot Heading (格式化為角度)
- Robot Position (X, Y 格式)
- Target (X, Y, Heading 格式)
- 場地繪圖功能（在 Dashboard 上顯示機器人和目標位置）

#### `sendDashboardTelemetry()` 方法簡化：
移除的內容：
- 位置誤差數據 (`errorX`, `errorY`, `errorHeading`)

保留的內容：
- **Dashboard Graph 數據**：
  - `currentX`, `currentY`, `currentHeading`
  - `targetX`, `targetY`, `targetHeading`
- **場地繪圖 (Field Overlay)**：
  - 當前位置（藍色機器人）
  - 目標位置（綠色機器人）

## 結果

現在 Swerve_Control 只顯示：

### Driver Station / Dashboard Telemetry 面板：
```
Mode: Field-Oriented
Heading: 45.0°
Position: X:1.23 Y:0.56
Target: X:2.00 Y:1.00 H:90.0°
Status: Running
```

### Dashboard Graph 面板：
- currentX
- currentY
- currentHeading
- targetX
- targetY
- targetHeading

### Dashboard Field 面板：
- 藍色機器人圖示（當前位置）
- 綠色機器人圖示（目標位置）
- 方向指示線

## 注意事項

如果需要調試特定功能，可以使用專用的調試 OpMode：
- `TurningPIDTuner.java` - 用於調整轉向 PID 參數
- 其他專門的測試程序

這樣可以保持 Swerve_Control 的遙測輸出簡潔清晰，適合比賽時使用。

