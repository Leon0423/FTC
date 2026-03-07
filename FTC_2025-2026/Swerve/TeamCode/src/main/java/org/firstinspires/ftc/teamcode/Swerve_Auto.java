package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

import java.util.List;


@Autonomous(name = "Swerve_Auto", group = "Auto")
public class Swerve_Auto extends LinearOpMode {

    // PID 控制器
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化 SwerveSubsystem
        SwerveSubsystem swerveSubsystem = new SwerveSubsystem(hardwareMap);

        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory - 第一段路徑：邊走邊轉
        //
        // 第一段路徑: (0,0) 面向 0° → (2,0) 面向 180°
        //
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(1, 0)
                ),
                new Pose2d(2, 0, Rotation2d.fromDegrees(180)),
                trajectoryConfig
        );

        // 3. Generate trajectory - 第二段路徑：走回原點
        //
        // 第二段路徑: (2,0) 面向 180° → (0,0) 面向 0°
        //
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2, 0, Rotation2d.fromDegrees(180)),
                List.of(
                        new Translation2d(1, 0)
                ),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig
        );

        // 3. Define PID controllers for tracking trajectory
        xController = new PIDController(
                AutoConstants.kPXController, 
                AutoConstants.kIXController, 
                AutoConstants.kDXController);
        yController = new PIDController(
                AutoConstants.kPYController, 
                AutoConstants.kIYController, 
                AutoConstants.kDYController);
        thetaController = new PIDController(
                AutoConstants.kPThetaController, 
                AutoConstants.kIThetaController, 
                AutoConstants.kDThetaController);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Trajectory 1 Duration", "%.2f seconds", trajectory1.getTotalTimeSeconds());
        telemetry.addData("Trajectory 2 Duration", "%.2f seconds", trajectory2.getTotalTimeSeconds());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 重置里程計到軌跡起始點
        swerveSubsystem.resetOdometry(trajectory1.getInitialPose());

        // 4. Follow trajectory 1 (第一段路徑)
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < trajectory1.getTotalTimeSeconds()) {
            // 獲取當前時間的目標狀態
            Trajectory.State targetState = trajectory1.sample(timer.seconds());
            Pose2d targetPose = targetState.poseMeters;

            // 獲取當前機器人位置
            Pose2d currentPose = swerveSubsystem.getPose();

            // 計算 PID 輸出
            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());

            // 計算角度誤差 (處理角度跨越 -π 到 π 的情況)
            double targetAngle = targetPose.getRotation().getRadians();
            double currentAngle = currentPose.getRotation().getRadians();
            double angleError = normalizeAngle(targetAngle - currentAngle);
            double rotSpeed = thetaController.calculate(0, -angleError);

            // 加入前饋速度
            xSpeed += targetState.velocityMetersPerSecond * targetPose.getRotation().getCos();
            ySpeed += targetState.velocityMetersPerSecond * targetPose.getRotation().getSin();

            // 轉換為 ChassisSpeeds 並計算模組狀態
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, currentPose.getRotation());

            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);

            // 更新里程計
            swerveSubsystem.periodic();

            // Telemetry
            telemetry.addData("Phase", "Trajectory 1");
            telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), trajectory1.getTotalTimeSeconds());
            telemetry.addData("Target", "X:%.2f Y:%.2f θ:%.1f°",
                    targetPose.getX(), targetPose.getY(), Math.toDegrees(targetAngle));
            telemetry.addData("Current", "X:%.2f Y:%.2f θ:%.1f°",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentAngle));
            telemetry.update();
        }

        // 5. 停止並等待 1 秒
        swerveSubsystem.stopModules();
        telemetry.addData("Status", "Waiting 1 second...");
        telemetry.update();
        sleep(1000);  // 等待 1 秒

        // 重置 PID 控制器 (避免積分累積)
        xController.reset();
        yController.reset();
        thetaController.reset();

        // 6. Follow trajectory 2 (第二段路徑：走回原點)
        timer.reset();

        while (opModeIsActive() && timer.seconds() < trajectory2.getTotalTimeSeconds()) {
            // 獲取當前時間的目標狀態
            Trajectory.State targetState = trajectory2.sample(timer.seconds());
            Pose2d targetPose = targetState.poseMeters;

            // 獲取當前機器人位置
            Pose2d currentPose = swerveSubsystem.getPose();

            // 計算 PID 輸出
            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());

            // 計算角度誤差
            double targetAngle = targetPose.getRotation().getRadians();
            double currentAngle = currentPose.getRotation().getRadians();
            double angleError = normalizeAngle(targetAngle - currentAngle);
            double rotSpeed = thetaController.calculate(0, -angleError);

            // 加入前饋速度
            xSpeed += targetState.velocityMetersPerSecond * targetPose.getRotation().getCos();
            ySpeed += targetState.velocityMetersPerSecond * targetPose.getRotation().getSin();

            // 轉換為 ChassisSpeeds 並計算模組狀態
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, currentPose.getRotation());

            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);

            // 更新里程計
            swerveSubsystem.periodic();

            // Telemetry
            telemetry.addData("Phase", "Trajectory 2 (Return)");
            telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), trajectory2.getTotalTimeSeconds());
            telemetry.addData("Target", "X:%.2f Y:%.2f θ:%.1f°",
                    targetPose.getX(), targetPose.getY(), Math.toDegrees(targetAngle));
            telemetry.addData("Current", "X:%.2f Y:%.2f θ:%.1f°",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentAngle));
            telemetry.update();
        }

        // 7. 停止所有模組
        swerveSubsystem.stopModules();

        telemetry.addData("Status", "Trajectory Complete!");
        telemetry.update();
    }

    /**
     * 將角度正規化到 -π 到 π 的範圍
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
