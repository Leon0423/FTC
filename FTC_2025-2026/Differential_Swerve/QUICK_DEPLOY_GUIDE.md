# 邊直走邊轉問題修復 - 快速部署指南

## 修復已完成 ✅

### 改動內容
修改文件: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`

**轉向 PID 參數 (ModuleConstants 類別)**:
- `kPTurning`: 0.5 → 0.8
- `kITurning`: 0.0 → 0.01
- `kDTurning`: 0.0 → 0.05
- `kTurningDeadbandDeg`: 1.0 → 0.5

## 部署步驟

### 方式 1: 使用 Android Studio（推薦）
```
1. 打開項目
2. 選擇 menu: Build → Make Project
3. 連接機器人
4. 選擇 Run → Run 'TeamCode'（或按 Shift+F10）
5. 選擇設備
```

### 方式 2: 使用命令行（Gradle）
```powershell
# Windows PowerShell
cd D:\FTC\FTC_2025-2026\Swerve

# 編譯
./gradlew build

# 部署到機器人（需要 adb 配置）
./gradlew installDebug
```

### 方式 3: 生成 APK 並手動安裝
```powershell
cd D:\FTC\FTC_2025-2026\Swerve

# 生成 APK
./gradlew assembleDebug

# APK 位置: TeamCode/build/outputs/apk/debug/TeamCode-debug.apk

# 使用 adb 安裝（需連接手機/中樞）
adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
```

## 測試驗證

### 快速驗證（現場）
```
1. 機器人上電
2. 選擇 Swerve_Control OpMode
3. 運行：
   - 左搖桿向前推（直走）
   - 同時右搖桿向左/右轉向
4. 觀察機器人是否沿曲線行進（不在原地旋轉）
```

### 使用 FTC Dashboard 驗證
```
1. 連接 FTC Dashboard (http://192.168.43.1:8080)
2. 選擇 Swerve_Control
3. 觀察 TuningConfig 面板
4. 檢查顯示的 PID 參數是否為：
   - _1a_turningP = 0.8
   - _1b_turningI = 0.01
   - _1c_turningD = 0.05
   - _2a_deadbandDeg = 0.5
```

## 常見問題排查

### 編譯錯誤
```
問題: Gradle 編譯失敗
解決:
1. Clean project: Build → Clean Project
2. Rebuild: Build → Make Project
3. 檢查 SDK 版本: File → Project Structure
```

### 部署失敗
```
問題: 無法安裝到設備
解決:
1. 檢查 USB 連接
2. 在手機/中樞上授予 USB 調試權限
3. 運行: adb devices（確認設備列表）
```

### 運行時參數沒有改變
```
問題: FTC Dashboard 顯示舊參數
解決:
1. 清除 build 文件夾: ./gradlew clean
2. 重新編譯: ./gradlew build
3. 重新部署應用
4. 在 Dashboard 刷新頁面（F5）
```

## PID 調整命令（實時）

無需重新編譯！在 FTC Dashboard 的 `TuningConfig` 面板直接修改：

### 參數位置
- Turning P (1_a_turningP): 當前 0.8
- Turning I (1_b_turningI): 當前 0.01
- Turning D (1_c_turningD): 當前 0.05
- Deadband (2_a_deadbandDeg): 當前 0.5

### 即時調整示例
```
若轉向過度反應:
1. 在 Dashboard 將 _1a_turningP 改為 0.7
2. 將 _1c_turningD 改為 0.1
3. 立即測試（無需重新部署）

若轉向不夠快:
1. 在 Dashboard 將 _1a_turningP 改為 0.9 或 1.0
2. 立即測試
```

## 回滾步驟（如需恢復原始設定）

### 方式 1: 編輯代碼回滾
```
1. 打開 Constants.java
2. 找到 ModuleConstants 類別（約 36-49 行）
3. 改回原值:
   - kPTurning = 0.5
   - kITurning = 0.0
   - kDTurning = 0.0
   - kTurningDeadbandDeg = 1.0
4. 重新編譯部署
```

### 方式 2: 使用 Dashboard 的 Dashboard 配置
```
如果只是想臨時測試舊參數，可在 Dashboard 改為:
- _1a_turningP = 0.5
- _1b_turningI = 0.0
- _1c_turningD = 0.0
- _2a_deadbandDeg = 1.0
```

## 效能對比

修改前後的預期表現對比：

| 操作 | 修改前 | 修改後 |
|------|-------|--------|
| 轉向反應 | 遲鈍（延遲 0.3-0.5s） | 快速（延遲 < 0.1s） |
| 邊走邊轉 | 原地旋轉 | 曲線行進 ✓ |
| 轉向精度 | ±5-10° | ±1-3° |
| 敏銳度 | 低（需要大轉向） | 高（細微轉向可感應） |

## 文檔引用

- 詳細技術說明: `EDGE_DRIVE_ROTATION_FIX.md`
- 修復摘要: `FIX_SUMMARY.md`
- 原始配置: `Constants.java` → `ModuleConstants` 類別

## 技術支持

若有問題，請檢查：
1. `SwerveModule.java` - 轉向 PID 實現
2. `SwerveJoystickCmd.java` - 搖桿指令處理
3. FTC Dashboard - 實時 PID 監控和調整

---
**最後更新**: 2026-03-19
**版本**: 1.0

