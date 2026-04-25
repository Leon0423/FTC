# Pinpoint 使用說明與問題檢查

本文件提供這個專案的 Pinpoint 快速上手流程，並附上可直接在場上執行的檢查步驟。

## 1) 啟用 Pinpoint

請先確認 `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`：

- `DriveConstants.USING_PINPOINT = true`
- `DriveConstants.kPinpointName = "pinpoint"`（需和 Robot Configuration 名稱一致）
- `DriveConstants.kPinpointXPodOffsetMM`、`DriveConstants.kPinpointYPodOffsetMM` 需填入實測值
- `DriveConstants.kPinpointXEncoderReversed`、`DriveConstants.kPinpointYEncoderReversed` 需依實際方向調整

目前專案已將 `DriveConstants.USING_PINPOINT` 設為 `true`。

## 2) 座標與方向基準

本專案既有註解約定：

- X 正向：前進
- Y 正向：右移
- Heading 正向：逆時針

本專案已將 `kPinpointYEncoderReversed = true`，將 Pinpoint Y 軸反向以符合隊內慣例：

- 前進時 X 應增加
- **右移時 Y 應增加**（已對齊本專案座標定義）

## 3) 目前已修正的重要問題

已修正 `SwerveSubsystem.zeroHeading()` 在 Pinpoint 模式下的行為：

- 修正前：呼叫 `zeroHeading()` 會用 `resetPosAndIMU()`，導致 X/Y 也被清零
- 修正後：`zeroHeading()` 只重設航向，不會改變當前 X/Y

修正位置：`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/SwerveSubsystem.java`

## 4) 上場前 60 秒檢查清單

建議直接執行 `6. PinpointQuickCheck`（`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Tuning/_6_PinpointQuickCheck.java`）。

1. 將機器人靜止放地上，進入 TeleOp。
2. 按一次 A（你的程式目前會呼叫 `zeroHeading()`）。
3. 不推車時確認 Heading 接近 `0`。
4. 手推機器人前進，確認 X 增加。
5. 手推機器人右移，確認 Y 的符號符合你隊內座標定義。
6. 手動旋轉機器人 90 度，確認 Heading 方向與數值變化正確。

## 5) 常見異常與處理

- Heading 漂移很大
  - 開始前務必靜止數秒，避免 IMU 校正品質不佳。
- 前進時 X 減少
  - 調整 `kPinpointXEncoderReversed`。
- 橫移方向顛倒
  - 調整 `kPinpointYEncoderReversed`，並重新核對你隊內 Y 軸定義。
- 定位有系統性偏差
  - 重新量測 `kPinpointXPodOffsetMM`、`kPinpointYPodOffsetMM`。

## 6) 建議測試順序

- 先做純手推定位驗證（不跑自動路徑）
- 再測短距離直線 Auto
- 最後測曲線軌跡

這樣能快速分離「感測座標問題」和「控制器/PID 問題」。


