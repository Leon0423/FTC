package org.firstinspires.ftc.teamcode.LATEST_TEST;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveDrive {
    // 硬體常數
    private static final double WHEEL_DIAMETER_MM = 72.0;
    private static final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;
    private static final double SERVO_RANGE_DEGREES = 300.0; // AXON MINI+ 通常是300度範圍

    // 死區設定
    private static final double DRIVE_DEADZONE = 0.05;
    private static final double TURN_DEADZONE = 0.05;

    // 硬體元件
    private final DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private final Servo frontLeftServo, frontRightServo, backLeftServo, backRightServo;
    private final IMU imu;

    // Module offset值
    private final double frontLeftOffset;
    private final double frontRightOffset;
    private final double backLeftOffset;
    private final double backRightOffset;

    public SwerveDrive(
            DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br,
            Servo fls, Servo frs, Servo bls, Servo brs,
            IMU imu,
            double flOffset, double frOffset, double blOffset, double brOffset) {

        this.frontLeftDrive = fl;
        this.frontRightDrive = fr;
        this.backLeftDrive = bl;
        this.backRightDrive = br;

        this.frontLeftServo = fls;
        this.frontRightServo = frs;
        this.backLeftServo = bls;
        this.backRightServo = brs;

        this.imu = imu;

        this.frontLeftOffset = flOffset;
        this.frontRightOffset = frOffset;
        this.backLeftOffset = blOffset;
        this.backRightOffset = brOffset;

        // 設定馬達方向
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // 設定馬達模式
        setAllDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // 設定所有驅動馬達模式
    private void setAllDriveMotorsMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    // 應用死區
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;
    }

    // 將角度轉換為Servo位置
    private double angleToServoPosition(double angle, double offset) {
        // 標準化角度到 0-360 範圍
        angle = (angle + offset) % 360;
        if (angle < 0) angle += 360;

        // 找出最短路徑
        double normalizedAngle = angle;
        if (angle > 180) {
            normalizedAngle = angle - 360;
        }

        // 將角度映射到servo範圍
        double servoRange = SERVO_MAX_POS - SERVO_MIN_POS;
        double position = (normalizedAngle / SERVO_RANGE_DEGREES) * servoRange + 0.5;

        return Range.clip(position, SERVO_MIN_POS, SERVO_MAX_POS);
    }

    // 主要驅動方法
    public void drive(double forward, double strafe, double rotate, boolean fieldCentric) {
        // 應用死區
        forward = applyDeadzone(forward, DRIVE_DEADZONE);
        strafe = applyDeadzone(strafe, DRIVE_DEADZONE);
        rotate = applyDeadzone(rotate, TURN_DEADZONE);

        // 如果所有輸入都在死區內，停止所有動作
        if (forward == 0 && strafe == 0 && rotate == 0) {
            stopAllMotors();
            return;
        }

        // 場地為中心的控制轉換
        if (fieldCentric) {
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double temp = forward * Math.cos(robotHeading) + strafe * Math.sin(robotHeading);
            strafe = -forward * Math.sin(robotHeading) + strafe * Math.cos(robotHeading);
            forward = temp;
        }

        // 計算每個模組的速度和角度
        double r = Math.hypot(forward, strafe);
        double robotAngle = Math.atan2(strafe, forward);

        // 計算每個輪子的數值
        double frontLeftSpeed = r + rotate;
        double frontRightSpeed = r - rotate;
        double backLeftSpeed = r + rotate;
        double backRightSpeed = r - rotate;

        // 標準化速度
        double max = Math.max(Math.abs(frontLeftSpeed), Math.max(Math.abs(frontRightSpeed),
                Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed))));

        if (max > 1.0) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        // 計算轉向角度（弧度轉度數）
        double angle = Math.toDegrees(robotAngle);

        // 設定Servo角度
        frontLeftServo.setPosition(angleToServoPosition(angle, frontLeftOffset));
        frontRightServo.setPosition(angleToServoPosition(angle, frontRightOffset));
        backLeftServo.setPosition(angleToServoPosition(angle, backLeftOffset));
        backRightServo.setPosition(angleToServoPosition(angle, backRightOffset));

        // 設定驅動馬達動力
        frontLeftDrive.setPower(frontLeftSpeed);
        frontRightDrive.setPower(frontRightSpeed);
        backLeftDrive.setPower(backLeftSpeed);
        backRightDrive.setPower(backRightSpeed);
    }

    // 停止所有馬達
    public void stopAllMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}