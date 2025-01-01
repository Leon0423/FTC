package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Claude_without_encoder")
public class Claude_without_encoder extends LinearOpMode {
    // 硬體宣告
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private ServoImplEx leftFrontServo = null;
    private ServoImplEx rightFrontServo = null;
    private ServoImplEx leftBackServo = null;
    private ServoImplEx rightBackServo = null;

    private IMU imu = null;

    // 常數定義
    private static final double DRIVE_SPEED = 0.8;
    private static final double TURN_SPEED = 0.5;
    private static final double DEAD_ZONE = 0.1;
    private static final double COUNTS_PER_SERVO_REV = 1425.1;  // Servo encoder的一圈計數

    // 追蹤Servo當前角度
    private double[] currentServoAngles = new double[4];
    private boolean[] wheelDirections = new boolean[4]; // true表示正向，false表示反向

    @Override
    public void runOpMode() {
        // 初始化硬體
        initializeHardware();

        // 等待開始
        waitForStart();

        while (opModeIsActive()) {
            // 獲取搖桿輸入
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // 獲取機器人當前朝向
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double robotAngle = orientation.getYaw(AngleUnit.RADIANS);

            // 計算每個輪子的驅動值和轉向角度
            double[] wheelSpeeds = new double[4];
            double[] wheelAngles = new double[4];

            if (Math.abs(drive) > DEAD_ZONE || Math.abs(strafe) > DEAD_ZONE || Math.abs(turn) > DEAD_ZONE) {
                // 計算移動方向和大小
                double r = Math.hypot(strafe, drive);
                double moveAngle = Math.atan2(strafe, drive) - robotAngle;

                // 計算每個輪子的目標角度
                wheelAngles[0] = Math.atan2(strafe + turn, drive + turn);
                wheelAngles[1] = Math.atan2(strafe + turn, drive - turn);
                wheelAngles[2] = Math.atan2(strafe - turn, drive + turn);
                wheelAngles[3] = Math.atan2(strafe - turn, drive - turn);

                // 為每個輪子計算最小轉向角度和方向
                for (int i = 0; i < 4; i++) {
                    double currentAngle = encoderToAngle(getServoEncoder(i));
                    double[] rotationResult = calculateMinimalRotation(currentAngle, wheelAngles[i]);
                    wheelAngles[i] = rotationResult[0];
                    wheelDirections[i] = rotationResult[1] > 0;

                    // 根據方向調整速度
                    wheelSpeeds[i] = r * (wheelDirections[i] ? 1.0 : -1.0);
                }

                // 正規化速度
                double max = Math.max(Math.max(Math.abs(wheelSpeeds[0]), Math.abs(wheelSpeeds[1])),
                        Math.max(Math.abs(wheelSpeeds[2]), Math.abs(wheelSpeeds[3])));
                if (max > 1.0) {
                    for (int i = 0; i < wheelSpeeds.length; i++) {
                        wheelSpeeds[i] /= max;
                    }
                }

                // 設置馬達速度和轉向
                setDriveMotors(wheelSpeeds);
                setServoAngles(wheelAngles);

                // 更新當前角度
                for (int i = 0; i < 4; i++) {
                    currentServoAngles[i] = wheelAngles[i];
                }
            } else {
                // 停止所有馬達
                stopMotors();
            }

            // 顯示遙測資料
            updateTelemetry(robotAngle, drive, strafe, turn);
        }
    }

    private void initializeHardware() {
        // 初始化驅動馬達
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // 設置馬達方向
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // 初始化轉向舵機
        leftFrontServo = hardwareMap.get(ServoImplEx.class, "left_front_servo");
        rightFrontServo = hardwareMap.get(ServoImplEx.class, "right_front_servo");
        leftBackServo = hardwareMap.get(ServoImplEx.class, "left_back_servo");
        rightBackServo = hardwareMap.get(ServoImplEx.class, "right_back_servo");

        // 啟用Servo encoder
        leftFrontServo.setPwmEnable();
        rightFrontServo.setPwmEnable();
        leftBackServo.setPwmEnable();
        rightBackServo.setPwmEnable();

        // 初始化IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        // 初始化方向陣列
        for (int i = 0; i < 4; i++) {
            wheelDirections[i] = true;
            currentServoAngles[i] = 0;
        }
    }

    private double[] calculateMinimalRotation(double currentAngle, double targetAngle) {
        double diff = ((targetAngle - currentAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

        // 如果需要轉動超過90度，則反轉方向
        if (Math.abs(diff) > Math.PI/2) {
            if (diff > 0) {
                diff -= Math.PI;
            } else {
                diff += Math.PI;
            }
            return new double[]{diff, -1.0}; // 返回角度和方向(-1表示反向)
        }
        return new double[]{diff, 1.0}; // 返回角度和方向(1表示正向)
    }

    private double encoderToAngle(double encoderValue) {
        return (encoderValue / COUNTS_PER_SERVO_REV) * 2 * Math.PI;
    }

    private double angleToEncoder(double angle) {
        return (angle / (2 * Math.PI)) * COUNTS_PER_SERVO_REV;
    }

    private double getServoEncoder(int index) {
        switch(index) {
            case 0: return leftFrontServo.getPosition() * COUNTS_PER_SERVO_REV;
            case 1: return rightFrontServo.getPosition() * COUNTS_PER_SERVO_REV;
            case 2: return leftBackServo.getPosition() * COUNTS_PER_SERVO_REV;
            case 3: return rightBackServo.getPosition() * COUNTS_PER_SERVO_REV;
            default: return 0;
        }
    }

    private void setDriveMotors(double[] speeds) {
        leftFrontDrive.setPower(speeds[0] * DRIVE_SPEED * (wheelDirections[0] ? 1.0 : -1.0));
        rightFrontDrive.setPower(speeds[1] * DRIVE_SPEED * (wheelDirections[1] ? 1.0 : -1.0));
        leftBackDrive.setPower(speeds[2] * DRIVE_SPEED * (wheelDirections[2] ? 1.0 : -1.0));
        rightBackDrive.setPower(speeds[3] * DRIVE_SPEED * (wheelDirections[3] ? 1.0 : -1.0));
    }

    private void setServoAngles(double[] angles) {
        leftFrontServo.setPosition(angleToEncoder(angles[0]) / COUNTS_PER_SERVO_REV);
        rightFrontServo.setPosition(angleToEncoder(angles[1]) / COUNTS_PER_SERVO_REV);
        leftBackServo.setPosition(angleToEncoder(angles[2]) / COUNTS_PER_SERVO_REV);
        rightBackServo.setPosition(angleToEncoder(angles[3]) / COUNTS_PER_SERVO_REV);
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void updateTelemetry(double robotAngle, double drive, double strafe, double turn) {
        telemetry.addData("Robot Angle", Math.toDegrees(robotAngle));
        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);

        // 添加Servo資訊
        telemetry.addData("LF Servo", "Angle: %.1f° Dir: %s",
                Math.toDegrees(currentServoAngles[0]),
                wheelDirections[0] ? "Forward" : "Reverse");
        telemetry.addData("RF Servo", "Angle: %.1f° Dir: %s",
                Math.toDegrees(currentServoAngles[1]),
                wheelDirections[1] ? "Forward" : "Reverse");
        telemetry.addData("LB Servo", "Angle: %.1f° Dir: %s",
                Math.toDegrees(currentServoAngles[2]),
                wheelDirections[2] ? "Forward" : "Reverse");
        telemetry.addData("RB Servo", "Angle: %.1f° Dir: %s",
                Math.toDegrees(currentServoAngles[3]),
                wheelDirections[3] ? "Forward" : "Reverse");

        telemetry.update();
    }
}