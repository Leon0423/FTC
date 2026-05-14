package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.SwerveModuleState;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Constants.IMU_NAME;
import static org.firstinspires.ftc.teamcode.Constants.LEFT_MOTOR_A;
import static org.firstinspires.ftc.teamcode.Constants.LEFT_MOTOR_A_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.LEFT_MOTOR_B;
import static org.firstinspires.ftc.teamcode.Constants.LEFT_MOTOR_B_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.MAX_DRIVE_SPEED_MPS;
import static org.firstinspires.ftc.teamcode.Constants.RIGHT_MOTOR_A;
import static org.firstinspires.ftc.teamcode.Constants.RIGHT_MOTOR_A_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.RIGHT_MOTOR_B;
import static org.firstinspires.ftc.teamcode.Constants.RIGHT_MOTOR_B_INVERTED;
import static org.firstinspires.ftc.teamcode.Constants.TRACK_WIDTH_METERS;

@SuppressWarnings("deprecation")
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule leftModule;
    private final SwerveModule rightModule;
    private final SwerveDriveKinematics kinematics;
    private final IMU imu;

    public SwerveSubsystem(HardwareMap hardwareMap) {
        leftModule = new SwerveModule(
                hardwareMap,
                LEFT_MOTOR_A,
                LEFT_MOTOR_B,
                LEFT_MOTOR_A_INVERTED,
                LEFT_MOTOR_B_INVERTED
        );
        rightModule = new SwerveModule(
                hardwareMap,
                RIGHT_MOTOR_A,
                RIGHT_MOTOR_B,
                RIGHT_MOTOR_A_INVERTED,
                RIGHT_MOTOR_B_INVERTED
        );

        kinematics = new SwerveDriveKinematics(
                new Translation2d(0.0, TRACK_WIDTH_METERS / 2.0),
                new Translation2d(0.0, -TRACK_WIDTH_METERS / 2.0)
        );

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
    }

    public void drive(double vxMetersPerSecond,
                      double vyMetersPerSecond,
                      double omegaRadiansPerSecond,
                      boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        vxMetersPerSecond,
                        vyMetersPerSecond,
                        omegaRadiansPerSecond,
                        getHeading()
                )
                : new ChassisSpeeds(
                        vxMetersPerSecond,
                        vyMetersPerSecond,
                        omegaRadiansPerSecond
                );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_DRIVE_SPEED_MPS);

        leftModule.setState(states[0]);
        rightModule.setState(states[1]);
    }

    public void stop() {
        leftModule.stop();
        rightModule.stop();
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(
                -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }

    public SwerveModuleState getLeftState() {
        return leftModule.getState();
    }

    public SwerveModuleState getRightState() {
        return rightModule.getState();
    }
}
