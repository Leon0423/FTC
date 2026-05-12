# 邊直走邊轉問題修復指南

## 問題描述
- **症狀**：左搖桿直走 + 右搖桿轉向時，機器人以車頭前方一個點旋轉（field mode），而不是邊走邊轉
- **根本原因**：轉向馬達的 PID 參數反應不夠快，導致輪子角度調整滯後，無法正確執行斜向運動

## 已執行的修復

### 1. 提升轉向 PID 參數（Constants.java）
已更新 `ModuleConstants` 類別中的參數：

```java
kPTurning = 0.8;        // 提升至 0.8 以加快初期反應速度
kITurning = 0.01;       // 改為 0.01 以消除穩態誤差
kDTurning = 0.05;       // 改為 0.05 以減少超調
kTurningDeadbandDeg = 0.5;  // 減至 0.5 度以提高轉向敏銳度
```

**修改前**：
- P = 0.5（太低）
- I = 0.0（無積分項）
- D = 0.0（無微分項）
- Deadband = 1.0°（太寬）

**修改後**：
- P = 0.8（更快的初期反應）
- I = 0.01（消除穩態誤差）
- D = 0.05（減少超調和震盪）
- Deadband = 0.5°（更敏銳的轉向）

## 測試步驟

### 第一步：編譯並部署
```bash
# 在工程目錄執行
./gradlew build
./gradlew assembleDebug
```

### 第二步：基礎功能測試
1. **轉向單獨測試**
   - 右搖桿向左/右轉向，觀察四個輪子是否能快速對齊
   - 應該看到輪子快速轉向，無明顯延遲

2. **直走單獨測試**
   - 左搖桿向前推，觀察機器人是否直線行進
   - 所有輪子應該對齊車頭方向

### 第三步：邊直走邊轉測試（主要測試）
1. 左搖桿向前推（直走）
2. 同時右搖桿向左或向右（轉向）
3. **預期行為**：
   - ✅ 機器人應該沿著曲線路徑行進（邊走邊轉）
   - ❌ 不應該在原地旋轉（pivot）
   - 曲線應該相對平滑，無抖動

### 第四步：使用 FTC Dashboard 觀察 PID 數據
1. 連接 FTC Dashboard
2. 查看 `TuningConfig` 面板
3. 觀察轉向 PID 參數（1_a, 1_b, 1_c）
4. 邊走邊轉時觀察：
   - **Target Angle** 和 **Current Angle** 的跟蹤誤差
   - 應該很小（< 5°）且反應迅速

## 進階調整（如果仍有問題）

### 如果轉向過度響應或抖動
- 降低 P 值（0.8 → 0.7 或 0.6）
- 增加 D 值（0.05 → 0.1 或 0.15）

### 如果轉向反應仍不夠快
- 增加 P 值（0.8 → 1.0）
- 檢查 `kTurningMotorGearRatio` 是否正確（應為 0.4）

### 如果某個輪子反應明顯滯後
- 使用 `_1b_TurningEncoderReverse.java` 檢查轉向編碼器方向
- 使用 `_1a_EncoderOffsetReader.java` 檢查偏移值是否準確

## 診斷命令

### 1. 檢查轉向馬達齒輪比
運行 `_3_MaxSpeedAngularTest.java`，觀察角速度是否合理

### 2. 檢查轉向編碼器反向
運行 `_1b_TurningEncoderReverse.java`，確認各輪轉向方向正確

### 3. 檢查編碼器偏移
運行 `_1a_EncoderOffsetReader.java`，讀取實時偏移值
運行 `_1c_AutoSetOffset.java`，自動校準並保存

### 4. 實時 PID 調整
在 FTC Dashboard 的 `TuningConfig` 面板中實時修改 PID 參數：
- `_1a_turningP` (原 kPTurning)
- `_1b_turningI` (原 kITurning)
- `_1c_turningD` (原 kDTurning)
- `_2a_deadbandDeg` (原 kTurningDeadbandDeg)

修改後無需重新部署，即可看到效果

## 預期改進效果

在新的 PID 參數下，應該觀察到：

1. **轉向響應更快**
   - 目標角度和當前角度的偏差 < 5°
   - 誤差消除時間 < 0.3 秒

2. **邊走邊轉更流暢**
   - 機器人沿平滑曲線行進
   - 無明顯的抖動或跳躍

3. **控制更精確**
   - 轉向死區減小，敏銳度提高
   - 細微的轉向指令能被響應

## 常見問題排查

### Q: 修改後還是在原地旋轉？
**A**: 
1. 確認已經重新編譯並部署
2. 檢查轉向編碼器反向設定是否正確（使用 `_1b_TurningEncoderReverse.java`）
3. 檢查轉向馬達是否能正常工作
4. 在 Dashboard 中觀察轉向馬達輸出功率是否正常（應在 ±0.5 ~ ±1.0 範圍）

### Q: 轉向抖動或過度響應？
**A**:
1. 在 Dashboard 中降低 `_1a_turningP` 至 0.6 或 0.5
2. 增加 `_1c_turningD` 至 0.1 或 0.15
3. 觀察抖動是否消退

### Q: 轉向仍然很慢？
**A**:
1. 在 Dashboard 中增加 `_1a_turningP` 至 1.0
2. 檢查 `kTurningMotorGearRatio` 是否正確
3. 檢查轉向馬達是否安裝正確

## 回滾方案

如果新參數導致問題，可快速回滾：

在 `Constants.java` 的 `ModuleConstants` 中改回原值：
```java
public static final double kPTurning = 0.5;
public static final double kITurning = 0.0;
public static final double kDTurning = 0.0;
public static final double kTurningDeadbandDeg = 1.0;
```

## 相關文件參考

- **Constants.java** - 全局常數配置
- **SwerveModule.java** - 單個斜輪模組的控制邏輯
- **SwerveJoystickCmd.java** - 搖桿指令處理
- **TuningConfig.java** - FTC Dashboard 實時調整參數

---
**修改日期**: 2026-03-19
**修改者**: AI Assistant
**版本**: 1.0

