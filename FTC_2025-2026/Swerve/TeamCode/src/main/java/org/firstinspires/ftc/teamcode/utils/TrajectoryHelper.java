package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

/**
 * 軌跡建立輔助工具類別
 * 提供使用公分為單位的便利方法
 *
 * ==================== 座標系統說明 ====================
 * X 軸: 正值 = 前進 (機器人前方)，負值 = 後退
 * Y 軸: 正值 = 右移，負值 = 左移
 * 角度: 正值 = 逆時針旋轉，負值 = 順時針旋轉
 * 單位: 公分 (cm)，角度為度 (degrees)
 * =====================================================
 */
public class TrajectoryHelper {

    // 公分轉公尺 (FTCLib 內部使用公尺)
    private static final double CM_TO_M = 0.01;

    /**
     * 將公分轉換為公尺
     * @param centimeters 公分值
     * @return 公尺值
     */
    public static double cm(double centimeters) {
        return centimeters * CM_TO_M;
    }

    /**
     * 建立 Pose2d (使用公分和度數)
     * @param xCm X 座標 (公分)
     * @param yCm Y 座標 (公分)
     * @param headingDeg 朝向角度 (度)
     * @return Pose2d 物件
     */
    public static Pose2d pose(double xCm, double yCm, double headingDeg) {
        return new Pose2d(cm(xCm), cm(yCm), Rotation2d.fromDegrees(headingDeg));
    }

    /**
     * 建立 Pose2d (使用公分，朝向為 0°)
     * @param xCm X 座標 (公分)
     * @param yCm Y 座標 (公分)
     * @return Pose2d 物件
     */
    public static Pose2d pose(double xCm, double yCm) {
        return pose(xCm, yCm, 0);
    }

    /**
     * 建立 Translation2d (使用公分)
     * @param xCm X 座標 (公分)
     * @param yCm Y 座標 (公分)
     * @return Translation2d 物件
     */
    public static Translation2d point(double xCm, double yCm) {
        return new Translation2d(cm(xCm), cm(yCm));
    }
}


