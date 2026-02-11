package org.firstinspires.ftc.teamcode;

public class SlewRateLimiter {

    private final double rateLimit; // 每秒最大變化量 (單位/秒)
    private final double maxDt = 0.1; // dt 上限（秒）
    private double prevVal;
    private long prevTime;
    private boolean firstRun;

    public SlewRateLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
        this.prevVal = 0;
        this.prevTime = System.currentTimeMillis();
        this.firstRun = true;
    }

    public double calculate(double input) {
        long currentTime = System.currentTimeMillis();

        // 首次呼叫時重置時間，避免累積過大的 dt
        if (firstRun) {
            prevTime = currentTime;
            prevVal = input;
            firstRun = false;
            return input;
        }

        double dt = (currentTime - prevTime) / 1000.0; // 轉換為秒

        // 限制 dt 上限，防止異常大的時間差
        if (dt > maxDt) {
            dt = maxDt;
        }

        prevTime = currentTime;

        // 計算允許的最大變化量
        double maxChange = rateLimit * dt;

        // 限制變化幅度
        double output = input;
        if (input > prevVal + maxChange) {
            output = prevVal + maxChange;
        } else if (input < prevVal - maxChange) {
            output = prevVal - maxChange;
        }

        prevVal = output;
        return output;
    }

    /**
     * 重置 SlewRateLimiter 狀態，供模式切換時使用
     */
    public void reset() {
        prevVal = 0;
        prevTime = System.currentTimeMillis();
        firstRun = true;
    }
}