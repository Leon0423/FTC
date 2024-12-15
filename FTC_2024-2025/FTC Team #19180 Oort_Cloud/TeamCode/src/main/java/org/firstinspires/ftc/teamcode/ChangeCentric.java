package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "RobotCentric_FieldCentric", group = "TeleOp")
public class ChangeCentric extends LinearOpMode {
    //變數設定

    private GamepadEx gamepadEx;
    private MecanumDrive mecanumDrive;
    BHI260IMU imu;
    boolean FIELD_CENTRIC = false;

    //創建物件
    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        //初始狀態設定
        while (opModeIsActive()) {
            //迴圈執行內容
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (!FIELD_CENTRIC) {
                mecanumDrive.driveRobotCentric(
                        gamepadEx.getLeftX(),
                        gamepadEx.getLeftY(),
                        gamepadEx.getRightX(),
                        false
                );
                telemetry.addData("RobotCentric", "True");
            } else {
                reset_heading();
                mecanumDrive.driveFieldCentric(
                        gamepadEx.getLeftX(),
                        gamepadEx.getLeftY(),
                        gamepadEx.getRightX(),
                        heading,
                        false
                );
                telemetry.addData("FieldCentric", "True");
                telemetry.addData("Heading", heading);
            }

            if(gamepad1.a){
                FIELD_CENTRIC = !FIELD_CENTRIC;
            }

            if (gamepad1.b){
                reset_heading();
            }

            telemetry.update();

            idle();
        }
    }

    // 建立函式
    private void init_hardware () {

        //設定MecanumDrive
        mecanumDrive = new MecanumDrive(
                new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_312)
        );

        gamepadEx = new GamepadEx(gamepad1);

        //TODO: 設定 IMU Direction
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        idle();
    }

    private void reset_heading(){
        imu.resetYaw();
    }
}
// 外面不可以寫程式喔!!!
