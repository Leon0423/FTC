// ...existing code...
    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / absoluteEncoder.getMaxVoltage();
        angle *= 2.0 * Math.PI;
        // 將伺服軸角度換算為模組角度；kTurningMotorGearRatio 定義為「模組轉/伺服轉」
        angle *= ModuleConstants.kTurningMotorGearRatio;
        angle -= absoluteEncoderOffsetRad;

        // 如果反向，先乘以 -1
        if (absoluteEncoderReversed) {
            angle = -angle;
        }

        // 標準化角度到 [-π, π] 範圍
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;

        return angle;
    }
// ...existing code...
