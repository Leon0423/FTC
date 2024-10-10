package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "AbsoluteDrive")
public class AbsoluteDrive extends LinearOpMode {
    //變數設定
    private DcMotor FL, FR, BL, BR;
    private Gamepad driveController;
    private MecanumDrive mecanumDrive;
    BHI260IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while (opModeIsActive()) {
            //迴圈執行內容
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            mecanumDrive.driveFieldCentric(
                    driveController.left_stick_y,
                    -driveController.left_stick_x,
                    -driveController.right_stick_x,
                    heading
            );

            idle();
        }
    }

    public void init_hardware() {
        //設定物件
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.resetDeviceConfigurationForOpMode();
        FR.resetDeviceConfigurationForOpMode();
        BL.resetDeviceConfigurationForOpMode();
        BR.resetDeviceConfigurationForOpMode();

        mecanumDrive = new MecanumDrive((Motor) FL, (Motor) FR, (Motor) BL, (Motor) BR);
        driveController = gamepad1;

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
    }
}