package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Claude", group="Drive")
public class Claude extends LinearOpMode {

    // 定義馬達和舵機
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Servo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;

    // 常數
    private final double DRIVE_GEAR_RATIO = 10.0; // TODO: 根據您的驅動齒輪比調整
    private final double WHEEL_DIAMETER_MM = 72.0; // TODO: 根據您的輪子直徑調整
    private final double COUNTS_PER_MOTOR_REV = 6000; // TODO: 根據您的馬達規格調整
    private final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_RATIO) / (WHEEL_DIAMETER_MM * Math.PI);

    @Override
    public void runOpMode() {
        // 初始化硬件
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        frontLeftSteer = hardwareMap.get(Servo.class, "front_left_steer");
        frontRightSteer = hardwareMap.get(Servo.class, "front_right_steer");
        backLeftSteer = hardwareMap.get(Servo.class, "back_left_steer");
        backRightSteer = hardwareMap.get(Servo.class, "back_right_steer");

        // 設置馬達模式
        setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // 計算每個模組的速度和角度
            double[] speeds = new double[4];
            double[] angles = new double[4];

            // 前左
            speeds[0] = Math.sqrt((strafe - rotate) * (strafe - rotate) + (drive + rotate) * (drive + rotate));
            angles[0] = Math.atan2(drive + rotate, strafe - rotate);

            // 前右
            speeds[1] = Math.sqrt((strafe + rotate) * (strafe + rotate) + (drive + rotate) * (drive + rotate));
            angles[1] = Math.atan2(drive + rotate, strafe + rotate);

            // 後左
            speeds[2] = Math.sqrt((strafe - rotate) * (strafe - rotate) + (drive - rotate) * (drive - rotate));
            angles[2] = Math.atan2(drive - rotate, strafe - rotate);

            // 後右
            speeds[3] = Math.sqrt((strafe + rotate) * (strafe + rotate) + (drive - rotate) * (drive - rotate));
            angles[3] = Math.atan2(drive - rotate, strafe + rotate);

            // 標準化速度
            double max = Math.max(Math.max(speeds[0], speeds[1]), Math.max(speeds[2], speeds[3]));
            if (max > 1.0) {
                for (int i = 0; i < 4; i++) {
                    speeds[i] /= max;
                }
            }

            // 設置馬達功率和舵機位置
            frontLeftDrive.setPower(speeds[0]);
            frontRightDrive.setPower(speeds[1]);
            backLeftDrive.setPower(speeds[2]);
            backRightDrive.setPower(speeds[3]);

            frontLeftSteer.setPosition(angles[0] / (2 * Math.PI) + 0.5);
            frontRightSteer.setPosition(angles[1] / (2 * Math.PI) + 0.5);
            backLeftSteer.setPosition(angles[2] / (2 * Math.PI) + 0.5);
            backRightSteer.setPosition(angles[3] / (2 * Math.PI) + 0.5);

            // 遙測
            telemetry.addData("Front Left", "Speed: %.2f, Angle: %.2f", speeds[0], Math.toDegrees(angles[0]));
            telemetry.addData("Front Right", "Speed: %.2f, Angle: %.2f", speeds[1], Math.toDegrees(angles[1]));
            telemetry.addData("Back Left", "Speed: %.2f, Angle: %.2f", speeds[2], Math.toDegrees(angles[2]));
            telemetry.addData("Back Right", "Speed: %.2f, Angle: %.2f", speeds[3], Math.toDegrees(angles[3]));
            telemetry.update();
        }
    }

    private void setDriveMotorModes(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }
}