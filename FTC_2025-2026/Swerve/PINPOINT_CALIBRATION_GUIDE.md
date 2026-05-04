# Pinpoint 完整校準指南

## 概述
Pinpoint 的三大核心參數決定定位精度：
1. **Pod 編碼器方向** (`kPinpointXEncoderReversed`, `kPinpointYEncoderReversed`)
2. **Pod 相對中心偏移** (`kPinpointXPodOffsetMM`, `kPinpointYPodOffsetMM`)
3. **Heading 準確度**（由 Pinpoint 內部陀螺儀決定，或配合 IMU）

---

## 階段 1：基礎連接確認（5 分鐘）

### 工具：`Swerve_Control` 或 `_6_PinpointQuickCheck`

**操作步驟：**
1. 執行 OpMode
2. 觀察 telemetry 顯示：
   - `Pinpoint Connected: YES` ✓
   - Pose 值是否實時更新
   - 推車時 X/Y 是否有變化

**檢查清單：**
- [ ] Pinpoint 顯示 `CONNECTED`
- [ ] 推車能改變 X/Y 值
- [ ] 沒有 hardwareMap 連結錯誤

❌ **如果連接失敗**：
```
1. 檢查 Constants.java 中 kPinpointName = "pinpoint"
   是否與 FTC SDK hardwareMap 名稱一致

2. 檢查 I2C 線路：
   - Pinpoint 必須插在 Control Hub 的某個 I2C port
   - 檢查 pod 編碼器線是否牢固

3. 重新啟動 Control Hub
```

---

## 階段 2：Pod 編碼器方向校正（5-10 分鐘）

### 工具：`Swerve_Control`

**操作步驟：**
1. 把機器人放在平坦場地中央
2. 執行 `Swerve_Control`
3. 用手推機器人，檢查方向符號

| 動作 | X 應該 | Y 應該 |
|------|--------|--------|
| 向前推（車頭方向） | **增加** ➡️ | 不變 |
| 向右推（垂直方向） | 不變 | **增加** ➡️** |
| 原地旋轉 | 不變 | 不變 |

**校正步驟：**
```
如果向前但 X 減少：
  → Constants.java 改 kPinpointXEncoderReversed = !kPinpointXEncoderReversed

如果向右但 Y 減少：
  → Constants.java 改 kPinpointYEncoderReversed = !kPinpointYEncoderReversed
```

✓ **確認無誤後**，進入階段 3

---

## 階段 3：Pod Offset 自動校準（10-15 分鐘）

### 工具：`_7_PinpointOffsetCalibration`

**原理：**
- 原地旋轉一圈回到初始方向
- Pinpoint 會因為 Pod 與旋轉中心距離而產生漂移
- 工具自動計算應調整的 offset 值

**操作步驟：**

1. **執行 OpMode**
   ```
   在 FTC Control Hub 選擇 "_7. PinpointOffsetCalibration"
   ```

2. **放置機器人**
   - 放在場地任意位置（最好靠近中央）
   - 確保表面平坦、沒有障礙物

3. **標記起點**
   ```
   按 A，telemetry 會顯示：
   >>> START MARKED <<<
   Initial X: ...
   Initial Y: ...
   Initial θ: ...
   ```

4. **推機器人旋轉一圈**
   - **用手推**機器人，使其旋轉約 360°
   - 回到原始朝向
   - 盡量保持位置不變（旋轉中心固定）

5. **記錄漂移**
   ```
   按 B，結果會顯示：
   
   Drift ΔX: 0.050 m (50 mm)
   Drift ΔY: -0.030 m (-30 mm)
   Total Drift: 0.058 m
   
   >> SUGGESTED OFFSET ADJUSTMENT <<
   Adjust kPinpointXPodOffsetMM by: -50 mm
   Adjust kPinpointYPodOffsetMM by: 30 mm
   ```

6. **更新 Constants.java**
   ```java
   // 當前值
   public static final double kPinpointXPodOffsetMM = 0.0;
   public static final double kPinpointYPodOffsetMM = 100.0;
   
   // 改為建議值（注意符號）
   public static final double kPinpointXPodOffsetMM = -50.0;  // ← 加上建議值
   public static final double kPinpointYPodOffsetMM = 100.0 + 30.0;  // ← 加上建議值
   ```

7. **重複測試**
   - 按 Y 重置
   - 重新測試直到漂移 < 5 cm

**收斂標準：**
```
✓ Excellent: Drift < 5 cm    ← 停留在這裡
⚠ Good:     Drift 5-10 cm    ← 可進行下一階段
✗ Needs:    Drift > 10 cm    ← 繼續調整
```

---

## 階段 4：Heading 準確度確認（5 分鐘）

### 工具：`_8_HeadingAccuracyTest`

**操作步驟：**

1. **執行 OpMode**
   ```
   選擇 "_8. HeadingAccuracyTest"
   ```

2. **推機器人到四個方向**
   ```
   1. 面向北 (前方)，按 A
      → Telemetry: ✓ North recorded
   
   2. 面向東 (右方)，按 B
      → Telemetry: ✓ East recorded
   
   3. 面向南 (後方)，按 X
      → Telemetry: ✓ South recorded
   
   4. 面向西 (左方)，按 Y
      → Telemetry: ✓ West recorded
   ```

3. **檢視結果**
   ```
   按右保險桿，顯示：
   
   North (0°)        Expected: 0.0° -> Measured: -1.2° | Error: -1.2°
   East (90°)        Expected: 90.0° -> Measured: 88.5° | Error: -1.5°
   South (180°)      Expected: 180.0° -> Measured: 179.2° | Error: -0.8°
   West (270°/-90°)  Expected: -90.0° -> Measured: -91.0° | Error: -1.0°
   
   Max Error: 1.5°
   Avg Error: 1.1°
   ✓ EXCELLENT: All errors < 5°
   ```

**評估標準：**
```
✓ Max Error < 5°  → Heading 精度優秀，可進行精密自動路徑
⚠ Max Error < 10° → 可接受，但大動作可能漂移
✗ Max Error > 10° → 需要檢查 Pinpoint 或 IMU 初始化
```

---

## 階段 5：前後左右往返測試（10 分鐘）

### 工具：`Swerve_Control`

**用途：** 驗證直線運動的精度損失

### 5.1 前後往返 1 米

```
1. 記錄初始位置：(X0=0, Y0=0)
   → 按 A (Swerve_Control 的 zeroHeading)

2. 向前推 1 米 (或用馬達轉 ~1 秒 @ 50% 速度)
   → 記錄位置：(X1, X0)
   → 預期 X1 ≈ 1.0 m

3. 向後推回原點
   → 記錄最終位置：(X2, Y2)
   → 預期 X2 ≈ 0 ± 0.05 m，Y2 ≈ 0 ± 0.05 m

   結論：如果誤差 > 5 cm → offset 或編碼器仍需微調
```

### 5.2 左右往返 1 米

```
1. 記錄初始位置：(X0, Y0)

2. 向右推 1 米
   → 記錄位置：(X1, Y1)
   → 預期 Y1 ≈ 1.0 m

3. 向左推回原點
   → 記錄最終位置：(X2, Y2)
   → 預期 Y2 ≈ 0 ± 0.05 m，X2 ≈ X0 ± 0.05 m

   結論：如果誤差 > 5 cm → offset 需微調
```

---

## 常見問題排查

### Q1: Heading 不變（一直停在 0°）
```
原因 1: Pinpoint 未初始化
→ 檢查 SwerveSubsystem 的 configurePinpoint() 是否被呼叫
→ 檢查 Constants.java 的 USING_PINPOINT = true

原因 2: 編碼器沒有旋轉信號
→ 檢查 Pod 是否真的在旋轉
→ 檢查 kPinpointXEncoderReversed/kPinpointYEncoderReversed 是否正確
```

### Q2: X/Y 漂移超過 10 cm
```
原因 1: Pod Offset 值太遠
→ 檢查機器人底盤上 Pod 的實際物理位置
→ 用直尺測量 Pod 到機器人中心的距離 (mm)

原因 2: 編碼器解析度設錯
→ 確認 pinpoint.setEncoderResolution() 的 Pod 型號
→ 預設是 goBILDA_4_BAR_POD，確認你的機器人用的是這款

原因 3: 旋轉中心移動了
→ 測試時確保只旋轉、不平移
```

### Q3: Offset 值一直在改變，收不斂
```
原因: 可能旋轉時有微小平移
→ 多測試幾次，取平均值
→ 或降速推車，確保純旋轉

如果值完全相反：
→ 檢查 kPinpointXEncoderReversed / kPinpointYEncoderReversed 是否反了
```

---

## 最終檢查清單

完成以下項目才視為校準成功：

- [ ] 階段 1：Pinpoint 成功連接
- [ ] 階段 2：編碼器方向正確（向前 X+，向右 Y+）
- [ ] 階段 3：Offset 漂移 < 5 cm
- [ ] 階段 4：Heading 誤差 < 5°（或 < 10° 也接受）
- [ ] 階段 5：往返 1 米誤差 < 5 cm

✓ **所有項目通過** → Pinpoint 已準備好用於自動模式

---

## 參考資料

- `Constants.java`：所有 Pinpoint 參數
- `SwerveSubsystem.java`：Pinpoint 初始化與讀取邏輯
- `_6_PinpointQuickCheck.java`：快速檢查工具
- `_7_PinpointOffsetCalibration.java`：自動 offset 校準
- `_8_HeadingAccuracyTest.java`：heading 準確度測試

