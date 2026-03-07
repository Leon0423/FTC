package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;
import org.firstinspires.ftc.teamcode.utils.TrajectoryFollower;

import static org.firstinspires.ftc.teamcode.utils.TrajectoryHelper.pose;
import static org.firstinspires.ftc.teamcode.utils.TrajectoryHelper.point;

import java.util.List;


@Autonomous(name = "Swerve_Auto", group = "Auto")
public class Swerve_Auto extends LinearOpMode {

    // ==================== 座標系統說明 ====================
    // X 軸: 正值 = 前進 (機器人前方)，負值 = 後退
    // Y 軸: 正值 = 右移，負值 = 左移
    // 角度: 正值 = 逆時針旋轉，負值 = 順時針旋轉
    // 單位: 公分 (cm)，角度為度 (degrees)
    // =====================================================

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化 SwerveSubsystem 和 TrajectoryFollower
        SwerveSubsystem swerveSubsystem = new SwerveSubsystem(hardwareMap);
        TrajectoryFollower follower = new TrajectoryFollower(swerveSubsystem, this);

        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DriveConstants.kDriveKinematics);

        // ========== 路徑 1: Curve Heading (曲線朝向) ==========
        // 機器人面向移動方向，像汽車一樣行駛
        // 路徑朝向跟隨路徑切線
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                pose(0, -50),
                List.of(
                        point(50, 25),
                        point(50, 75)
                ),
                pose(25, 50),
                trajectoryConfig
        );

        // ========== 路徑 2: Linear Heading (線性朝向插值) ==========
        // 只定義位置，機器人朝向由 followTrajectoryLinearHeading 控制
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                pose(25, 50),
                List.of(
                        point(30, 80)
                ),
                pose(0, 25),
                trajectoryConfig
        );

        // ========== 路徑 3: Constant Heading (固定朝向) ==========
        // 只定義位置，機器人朝向由 followTrajectoryConstantHeading 控制
        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
                pose(0, 25),
                List.of(),
                pose(50, 25),
                trajectoryConfig
        );


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Path 1 (Curve)", "%.2f sec", trajectory1.getTotalTimeSeconds());
        telemetry.addData("Path 2 (Linear)", "%.2f sec", trajectory2.getTotalTimeSeconds());
        telemetry.addData("Path 3 (Constant)", "%.2f sec", trajectory3.getTotalTimeSeconds());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 重置里程計到軌跡起始點
        swerveSubsystem.resetOdometry(trajectory1.getInitialPose());

        // ========== 執行路徑 1: Curve Heading ==========
        // 機器人朝向 = 跟隨路徑切線方向
        follower.followTrajectoryCurveHeading(trajectory1, "Path 1: Curve Heading", 0.5);

        sleep(500);

        // ========== 執行路徑 2: Linear Heading ==========
        // 機器人朝向 = 從 0° 漸變到 90°，同時額外旋轉 2 圈
        follower.followTrajectoryLinearHeading(trajectory2, "Path 2: Linear Heading", 0, 90, 0.3, 2);

        sleep(500);

        // ========== 執行路徑 3: Constant Heading ==========
        // 機器人朝向 = 固定保持 -135°
        follower.followTrajectoryConstantHeading(trajectory3, "Path 3: Constant Heading", -135, 0.5);

        // 完成
        telemetry.addData("Status", "All Trajectories Complete!");
        telemetry.addData("=== 三種 Heading 模式示範 ===", "");
        telemetry.addData("1. Curve", "面向移動方向");
        telemetry.addData("2. Linear", "朝向漸變 (0° → 90°)");
        telemetry.addData("3. Constant", "固定朝向 (-135°)");
        telemetry.update();
    }
}
