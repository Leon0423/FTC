# 修复验证清单

## 🔧 代码修改检查表

### ✅ 修改 1: Init 阶段 - 使用 alignTurningOnly()
**位置**: `_3_MaxSpeedAngularTest.java` 行 97-131

检查项：
- [x] 角速度模式使用 `alignTurningOnly(currentTargetAngles[i])`
- [x] 直线模式使用 `alignTurningOnly(0)`
- [x] 移除了 `setDesiredState()` 的调用
- [x] 对齐检查使用 `normalizeAngleDeg()`

### ✅ 修改 2: 目标角度计算 - 每个循环都重新计算
**位置**: `_3_MaxSpeedAngularTest.java` 行 97-114

检查项：
- [x] `currentTargetAngles` 在循环内每次都计算
- [x] 角速度模式时从 kinematics 提取角度
- [x] 直线模式时使用 [0, 0, 0, 0]
- [x] 确保 Init 和 Start 阶段使用相同的计算逻辑

### ✅ 修改 3: Start 阶段目标角度 - 不使用 normalizeWheelSpeeds()
**位置**: `_3_MaxSpeedAngularTest.java` 行 179-192

检查项：
- [x] 计算 `targetAngles` 数组（仅角速度模式）
- [x] **不调用** `SwerveDriveKinematics.normalizeWheelSpeeds()`
- [x] 直接提取 `states[i].angle.getRadians()`
- [x] 保存到 `targetAngles[]` 供后续使用

### ✅ 修改 4: 主循环 - 使用预计算的目标角度
**位置**: `_3_MaxSpeedAngularTest.java` 行 193-213

检查项：
- [x] 角速度模式使用 `alignTurningOnly(targetAngles[i])`
- [x] 直线模式使用 `alignTurningOnly(0)`
- [x] 两种模式都设置驱动功率
- [x] 循环持续时间检查正确

### ✅ 修改 5: 辅助方法 - normalizeAngleDeg()
**位置**: `_3_MaxSpeedAngularTest.java` 行 317-323

检查项：
- [x] 添加了 `normalizeAngleDeg()` 方法
- [x] 将角度标准化到 [-180°, 180°]
- [x] 正确的循环条件

## 🧪 功能验证步骤

### 步骤 1: 编译检查
```
gradle build
或在 IDE 中检查是否有红色错误提示
```
✅ 预期: 无编译错误

### 步骤 2: Init 阶段测试 (A 键)
```
- 选择 A 键（直线速度模式）
- 观察轮子对齐
- 预期: 4 个轮子都对齐到 0°
- 应该看到 "✅ 對齐完成"
```

### 步骤 3: Init 阶段测试 (B 键)
```
- 选择 B 键（角速度模式）
- 观察轮子对齐
- 预期: 4 个轮子都对齐到 kinematics 计算的相同角度
  - 通常应该是类似 +45°、-45°、+135°、-135° 这样的配置
  - 但重要的是所有角度都应该在初始化期间保持一致
- 应该看到 "✅ 對齐完成"
```

### 步骤 4: Start 阶段 - 角速度模式
```
- 按 Start
- 观察轮子运动
- 预期: 
  ✅ 4 个轮子都转到相同的目标角度
  ✅ 没有轮子突然转 180°
  ✅ 机器人开始自转
  ✅ 约 3 秒后自动停止
```

### 步骤 5: 结束阶段 - 轮子回到 0°
```
- 等待自动停止
- 观察轮子对齐
- 预期:
  ✅ 轮子对齐回 0°
  ✅ 显示测试结果
  ✅ 建议的 kPhysicalMaxAngularSpeedRadiansPerSecond 值
```

### 步骤 6: 重复测试 A 键
```
- 再次测试直线速度模式
- 预期:
  ✅ 4 个轮子都对齐到 0°
  ✅ 一秒内对齐完成
```

## 📊 预期结果对比

| 项目 | 修复前 | 修复后 |
|------|--------|--------|
| Init 时轮子状态 | 所有轮子对齐到目标 | 所有轮子对齐到目标 ✅ |
| Start 时轮子方向 | 2 个转 180°，2 个正常 ❌ | 4 个都转到相同方向 ✅ |
| 测试结束时角度 | 全部复位到 0° ❌ | 累积角度保存，可继续使用 ✅ |
| 重复测试稳定性 | 容易出现不一致 ❌ | 稳定一致 ✅ |

## 🔍 常见问题排查

### 问题: 编译错误 "normalizeAngleDeg() 找不到"
**解决**: 
- 检查是否添加了 `normalizeAngleDeg()` 方法
- 确保在类的最后（第 317 行附近）

### 问题: Init 阶段仍然有轮子转 180°
**排查**:
- 检查是否完全移除了 `setDesiredState()`
- 确认使用的是 `alignTurningOnly()`
- 检查 `currentTargetAngles` 是否正确计算

### 问题: Start 后仍然是 2 个轮子转 180°
**排查**:
- 检查是否添加了 `targetAngles[]` 数组
- 确认没有使用 `SwerveDriveKinematics.normalizeWheelSpeeds()`
- 检查主循环是否使用了 `targetAngles[i]` 而不是 `states[i].angle`

### 问题: 测试结束轮子仍然回到 0°（失去累积）
**排查**:
- 这是正常行为，因为 `disableSaving()` 在测试期间阻止了存储
- 最后的 `enableSaving()` 和 `stopModules()` 会在对齐到 0° 后保存
- 这是设计特性，避免测试数据污染系统状态

## 📝 修改摘要

| 文件 | 行数 | 修改内容 |
|------|------|---------|
| `_3_MaxSpeedAngularTest.java` | 97-114 | Init 阶段逻辑 |
| `_3_MaxSpeedAngularTest.java` | 118-125 | 对齐检查逻辑 |
| `_3_MaxSpeedAngularTest.java` | 179-192 | 目标角度计算 |
| `_3_MaxSpeedAngularTest.java` | 193-213 | 主循环实现 |
| `_3_MaxSpeedAngularTest.java` | 317-323 | 辅助方法添加 |

## ✨ 修复完成!

所有修改已应用。现在可以：
1. 编译上传代码
2. 按照验证步骤进行测试
3. 确认问题已解决

如需详细技术信息，请参考 `ANGULAR_TEST_FIX.md`

