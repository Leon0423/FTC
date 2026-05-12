# MaxSpeedAngularTest 问题修复说明

## 问题描述
在运行 `_3_MaxSpeedAngularTest` 角速度测试模式时，出现以下问题：
1. **Init 阶段**：按 B 键，轮子对齐到定点（如 45°）
2. **Start 阶段**：有两颗轮子突然转 180°，另外两颗正常  
3. **测试结束**：所有轮子都复位到 0°（失去之前的累积角度）

## 根本原因

### 问题源头
在 `SwerveModule.setDesiredState()` 方法中，使用了 `SwerveModuleState.optimize()` 来优化轮子转向：

```java
state = SwerveModuleState.optimize(state, getState().angle);
```

**optimize() 的作用**：为了避免轮子转超过 90°，它会自动选择最短路径：
- 如果目标是 45°，当前是 0°，直接转 45°
- 如果目标是 45°，当前是 225°，则反向转（将 45° 改为 225°），这样只转 0°

### 累积角度追踪问题
`SwerveModule` 使用累积角度追踪（delta-based tracking）来记录轮子的旋转历史：
- 当不同的轮子采用不同的等价角度（如一个 45°，一个 225°）时
- 它们的累积角度记录会不一致
- 导致轮子在回到 0° 时路径不一致

## 解决方案

### 修改 1: Init 阶段改用 alignTurningOnly()

**文件**: `_3_MaxSpeedAngularTest.java` (行 97-131)

从 `setDesiredState()` 改为 `alignTurningOnly()`：

```java
// 改变前（会调用 optimize）
swerve.getFrontLeft().setDesiredState(new SwerveModuleState(0.001, new Rotation2d(angle45)));

// 改变后（不调用 optimize）
swerve.getFrontLeft().alignTurningOnly(angle45);
```

**优点**：
- `alignTurningOnly()` 直接对齐到指定角度，不经过 optimize
- 所有轮子都对齐到完全相同的物理角度
- 累积角度追踪保持一致

### 修改 2: Start 阶段提前计算目标角度

**文件**: `_3_MaxSpeedAngularTest.java` (行 179-192)

```java
// 改变前
double[] targetAngles = null;
if (angularMode) {
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1.0);
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);  // ← 这个会改变角度！
    // 在循环中使用 states[i].angle.getRadians()
}

// 改变后
double[] targetAngles = null;
if (angularMode) {
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1.0);
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    // 不使用 normalizeWheelSpeeds！直接提取角度
    targetAngles = new double[4];
    targetAngles[0] = states[0].angle.getRadians();
    targetAngles[1] = states[1].angle.getRadians();
    targetAngles[2] = states[2].angle.getRadians();
    targetAngles[3] = states[3].angle.getRadians();
}
```

### 修改 3: 主循环使用一致的目标角度

**文件**: `_3_MaxSpeedAngularTest.java` (行 196-213)

```java
// 改变前
swerve.getFrontLeft().alignTurningOnly(states[0].angle.getRadians());
// ...

// 改变后
swerve.getFrontLeft().alignTurningOnly(targetAngles[0]);
```

**重要**：
- Init 和 Start 阶段对齐的角度来自相同的 kinematics 计算
- 不再使用 `normalizeWheelSpeeds()`，避免隐式的角度修改

### 修改 4: 改进对齐检查逻辑

**文件**: `_3_MaxSpeedAngularTest.java` (行 118-125)

从使用绝对值改为使用角度归一化：

```java
// 改变前（会将 -45° 和 45° 都视为对齐）
boolean flOk = Math.abs(Math.abs(flDeg) - targetDeg) < 5;

// 改变后（正确处理 ±180° 环绕）
boolean flOk = Math.abs(normalizeAngleDeg(flDeg - flTargetDeg)) < 5;
```

## 修改文件清单
- `_3_MaxSpeedAngularTest.java` ✅
  - Init 阶段改用 `alignTurningOnly()` 
  - 提前计算目标角度数组
  - 改进对齐检查逻辑
  - 添加 `normalizeAngleDeg()` 辅助方法

## 测试步骤

### 验证修复
1. 上传代码到机器人
2. 打开 FTC Driver Station
3. 选择 "3. MaxSpeedAngularTest" OpMode
4. **Init 阶段** (按 B 键启用角速度模式):
   - 观察轮子对齐状态（显示 "✅ 對齐完成，可以按 Start！"）
   - 所有 4 个轮子应该对齐到相同的目标角度
5. **Press Start**:
   - 观察轮子方向（应该全部转到相同角度，不应有 2 个突然转 180°）
   - 轮子应该以配置的功率旋转（推动机器人转圈）
6. **测试完成** (3 秒后自动停止):
   - 轮子应该对齐回 0°
   - 最后显示测试结果和建议的常数值

### 预期改进
- ❌ **问题**：两颗轮子转 180° → ✅ **修复**：四颗轮子同时转到相同目标角度
- ❌ **问题**：测试结束后全部恢复 0° → ✅ **修复**：累积角度正确保存，可用于后续测试
- ❌ **问题**：Init 和 Start 不同步 → ✅ **修复**：使用相同的 kinematics 计算结果

## 技术细节

### 为什么要避免 optimize()?

`SwerveModuleState.optimize()` 本来是为了避免轮子超过 90° 转向。但在我们的场景中：
- 我们明确要求特定的转向角度（从 kinematics 计算）
- 轮子进行持续的累积角度追踪
- 使用 optimize 会导致同一目标角度被转换为不同的等价表示

### alignTurningOnly() 的优势

```java
public void alignTurningOnly(double targetRad) {
    currentAngle = getTurningPosition();
    targetAngle = targetRad;
    
    Error = normalizeAngle(targetAngle - currentAngle);
    // PID 控制轮子转向，直到到达 targetRad
    // 不经过 optimize，完全一致的角度追踪
}
```

## 相关文件

- `Constants.java` - 包含 kinematics 配置
- `SwerveModule.java` - 轮子模块实现（alignTurningOnly 定义）
- `SwerveSubsystem.java` - Swerve 子系统
- `_2_DriveMotorDirectionTester.java` - 驱动方向测试（参考）
- `_4_TurningPIDTuner.java` - 转向 PID 调整（参考）

## 常见问题

**Q: 为什么 init 阶段对齐角度后还要在 Start 再对齐一次？**
A: Init 阶段是准备阶段，确保轮子在物理上对齐。Start 后的循环是持续给命令确保轮子保持对齐，因为轮子可能因为物理因素（如摩擦力变化）而微微漂移。

**Q: normalizeAngleDeg() 的作用是什么？**
A: 将角度差归一化到 [-180°, 180°] 范围，这样才能正确比较两个角度是否接近（例如 -175° 和 175° 其实只差 10°，不是 350°）。

**Q: 为什么不能用 normalizeWheelSpeeds()？**
A: `normalizeWheelSpeeds()` 在轮速超过最大值时，会自动调整所有轮速和角度。这个角度调整会破坏我们的累积角度追踪一致性。


