package org.firstinspires.ftc.teamcode.Swerve;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerve.commands.SwerveJoystickCmd;
import org.firstinspires.ftc.teamcode.Swerve.subsystems.SwerveSubsystem;

@TeleOp(name = "Swerve_Control", group = "TeleOp")
public class Swerve_Control extends LinearOpMode {

    private SwerveSubsystem swerveSubsystem;

    private GamepadEx driverGamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        // 傳入 hardwareMap 來初始化 subsystem
        swerveSubsystem = new SwerveSubsystem(hardwareMap);

        // 將標準 gamepad1 包裝成 GamepadEx
        driverGamepad = new GamepadEx(gamepad1);

        // 創建指令一次即可
        SwerveJoystickCmd joystickCmd = new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX(),
                () -> !driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
        );

        configureButtonBindings();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 執行指令
            joystickCmd.execute();

            // 更新 subsystem
            swerveSubsystem.periodic();

            swerveSubsystem.updateTelemetry(telemetry);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        swerveSubsystem.stopModules();
    }

    private void configureButtonBindings(){
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> swerveSubsystem.zeroHeading());
    }

}
