package org.firstinspires.ftc.teamcode.Swerve;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerve.commands.SwerveJoystickCmd;
import org.firstinspires.ftc.teamcode.Swerve.subsystems.SwerveSubsystem;

@TeleOp(name = "Swerve_Control", group = "TeleOp")
public class Swerve_Control extends LinearOpMode {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private GamepadEx driverGamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        // 將標準 gamepad1 包裝成 GamepadEx
        driverGamepad = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                    swerveSubsystem,
                    () -> -driverGamepad.getLeftY(),
                    () -> driverGamepad.getLeftX(),
                    () -> driverGamepad.getRightX(),
                    () -> !driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)));


            configureButtonBindings();

            telemetry.addData("Status", "Running");
            telemetry.update();
        }


    }

    private void configureButtonBindings(){
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> swerveSubsystem.zeroHeading());
    }

}
