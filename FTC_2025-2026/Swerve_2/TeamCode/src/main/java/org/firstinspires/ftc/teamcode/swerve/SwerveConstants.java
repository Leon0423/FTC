package org.firstinspires.ftc.teamcode.swerve;

public class SwerveConstants {

    // =========================
    // Hardware names
    // =========================
    public static final String FR_DRIVE = "FR";
    public static final String FL_DRIVE = "FL";
    public static final String BR_DRIVE = "BR";
    public static final String BL_DRIVE = "BL";

    public static final String FR_TURN = "FRTurn";
    public static final String FL_TURN = "FLTurn";
    public static final String BR_TURN = "BRTurn";
    public static final String BL_TURN = "BLTurn";

    public static final String FR_ANALOG = "FREncoder";
    public static final String FL_ANALOG = "FLEncoder";
    public static final String BR_ANALOG = "BREncoder";
    public static final String BL_ANALOG = "BLEncoder";

    // =========================
    // Direction config
    // =========================
    public static boolean FR_DRIVE_REVERSED = true;
    public static boolean FL_DRIVE_REVERSED = false;
    public static boolean BR_DRIVE_REVERSED = true;
    public static boolean BL_DRIVE_REVERSED = false;

    public static boolean FR_TURN_REVERSED = false;
    public static boolean FL_TURN_REVERSED = false;
    public static boolean BR_TURN_REVERSED = false;
    public static boolean BL_TURN_REVERSED = false;

    // 如果某顆 analog 角度方向相反，改成 true
    public static boolean FR_ANALOG_REVERSED = false;
    public static boolean FL_ANALOG_REVERSED = false;
    public static boolean BR_ANALOG_REVERSED = false;
    public static boolean BL_ANALOG_REVERSED = false;

    // =========================
    // Drive config
    // =========================
    public static double DRIVE_MOTOR_TICKS_PER_REV = 28.0; // 改成你的驅動馬達 encoder 值
    public static double DRIVE_GEAR_RATIO = 8.8;           // motor : wheel
    public static double WHEEL_DIAMETER_M = 0.096;

    // =========================
    // Robot geometry
    // =========================
    public static double TRACK_WIDTH = 0.32; // m
    public static double WHEEL_BASE = 0.32;  // m

    // =========================
    // TeleOp limits
    // =========================
    public static double MAX_TRANSLATION = 0.55;
    public static double MAX_ROTATION = 0.45;

    // =========================
    // Steering PID
    // =========================
    public static double STEER_kP = 0.010;
    public static double STEER_kI = 0.0;
    public static double STEER_kD = 0.0;
    public static double STEER_kF = 0.0;

    public static double STEER_MAX_POWER = 0.55;
    public static double STEER_TOLERANCE_DEG = 2.5;

    public static double DRIVE_SCALE_IF_ERROR_GT_20 = 0.5;
    public static double DRIVE_SCALE_IF_ERROR_GT_40 = 0.2;

    // =========================
    // Analog angle conversion
    // 預設先用 0~maxVoltage 對應 0~360 deg
    // 若你實測不是完整 360，可後續再調
    // =========================
    public static double ANALOG_FULL_SCALE_DEG = 360.0;

    // =========================
    // Forward offsets
    // 當輪子真正朝車頭時，該顆 analog 對應的角度
    // 先用 calibration mode 量出來
    // =========================
    public static double FR_FORWARD_OFFSET_DEG = 111.71;
    public static double FL_FORWARD_OFFSET_DEG = 177.83;
    public static double BR_FORWARD_OFFSET_DEG = 223.96;
    public static double BL_FORWARD_OFFSET_DEG = 282.11;
}