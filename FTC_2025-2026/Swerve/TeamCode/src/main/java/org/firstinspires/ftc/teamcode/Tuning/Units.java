package org.firstinspires.ftc.teamcode.Tuning;

/**
 * 單位轉換工具類
 */
public final class Units {

    /**
     * 將英寸轉換為公尺
     * @param inches 英寸值
     * @return 公尺值
     */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    /**
     * 將公尺轉換為英寸
     * @param meters 公尺值
     * @return 英寸值
     */
    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    /**
     * 將度數轉換為弧度
     * @param degrees 度數值
     * @return 弧度值
     */
    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * 將弧度轉換為度數
     * @param radians 弧度值
     * @return 度數值
     */
    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }
}
