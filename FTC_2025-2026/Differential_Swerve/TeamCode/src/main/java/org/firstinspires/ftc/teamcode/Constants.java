package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final String LEFT_MOTOR_A = "leftMotorA";
    public static final String LEFT_MOTOR_B = "leftMotorB";
    public static final String RIGHT_MOTOR_A = "rightMotorA";
    public static final String RIGHT_MOTOR_B = "rightMotorB";
    public static final String IMU_NAME = "imu";

    public static final double TRACK_WIDTH_METERS = 0.30;
    public static final double WHEEL_DIAMETER_METERS = 0.04;

    public static final double TICKS_PER_MOTOR_REV = 28.0;
    // Ratios are motor rotations per one output rotation.
    public static final double STEER_GEAR_RATIO = 1.0;
    public static final double DRIVE_GEAR_RATIO = 1.0;

    public static final double TICKS_TO_RADIANS_STEER =
            2.0 * Math.PI / (TICKS_PER_MOTOR_REV * STEER_GEAR_RATIO);
    public static final double TICKS_TO_METERS_DRIVE =
            Math.PI * WHEEL_DIAMETER_METERS / (TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO);

    public static final double DIFFERENTIAL_STEER_POWER_SCALE = STEER_GEAR_RATIO;
    public static final double DIFFERENTIAL_DRIVE_POWER_SCALE = DRIVE_GEAR_RATIO;

    public static final double MAX_DRIVE_SPEED_MPS = 1.5;
    public static final double MAX_ROTATION_SPEED_RPS = Math.PI;
    public static final double MODULE_OPTIMIZE_MIN_SPEED_MPS = 0.05;

    public static final double STEER_KP = 0.0;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.0;

    public static final double STEER_TOLERANCE_RADIANS = Math.max(
            Math.toRadians(2.0),
            TICKS_TO_RADIANS_STEER
    );

    public static final boolean LEFT_MOTOR_A_INVERTED = false;
    public static final boolean LEFT_MOTOR_B_INVERTED = true;
    public static final boolean RIGHT_MOTOR_A_INVERTED = false;
    public static final boolean RIGHT_MOTOR_B_INVERTED = true;

    public static final boolean RESET_ENCODERS_ON_INIT = true;

    public static final String TEST_MODULE_MOTOR_A = LEFT_MOTOR_A;
    public static final String TEST_MODULE_MOTOR_B = LEFT_MOTOR_B;
    public static final boolean TEST_MODULE_MOTOR_A_INVERTED = LEFT_MOTOR_A_INVERTED;
    public static final boolean TEST_MODULE_MOTOR_B_INVERTED = LEFT_MOTOR_B_INVERTED;

    public static final double TEST_MODULE_MANUAL_POWER_SCALE = 0.45;
    public static final double TEST_MODULE_CLOSED_LOOP_DRIVE_POWER = 0.25;
    public static final double TEST_MODULE_STICK_DEADBAND = 0.05;
    public static final double TEST_MODULE_RIGHT_TRIGGER_DEADBAND = 0.05;
    public static final double TEST_MODULE_TARGET_ANGLE_A_RADIANS = 0.0;
    public static final double TEST_MODULE_TARGET_ANGLE_B_RADIANS = Math.PI / 2.0;
    public static final double TEST_MODULE_TARGET_ANGLE_X_RADIANS = Math.PI;
}
