package org.firstinspires.ftc.teamcode.LATEST_TEST;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Swerve Drive Test")
public class Swerve_Drive_Test extends OpMode {
    private SwerveDrive swerveDrive;

    @Override
    public void init() {
        // 初始化硬體映射
        DcMotor fl = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor fr = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor bl = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor br = hardwareMap.get(DcMotor.class, "backRight");

        Servo fls = hardwareMap.get(Servo.class, "frontLeftServo");
        Servo frs = hardwareMap.get(Servo.class, "frontRightServo");
        Servo bls = hardwareMap.get(Servo.class, "backLeftServo");
        Servo brs = hardwareMap.get(Servo.class, "backRightServo");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        // TODO: 創建SwerveDrive實例（offset值需要根據實際情況調整）
        swerveDrive = new SwerveDrive(
                fl, fr, bl, br,
                fls, frs, bls, brs,
                imu,
                0.0, 0.0, 0.0, 0.0
        );
    }

    @Override
    public void loop() {
        // 讀取搖桿輸入
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // 控制Swerve驅動
        swerveDrive.drive(forward, strafe, rotate, true); // true表示使用場地為中心的控制
    }
}