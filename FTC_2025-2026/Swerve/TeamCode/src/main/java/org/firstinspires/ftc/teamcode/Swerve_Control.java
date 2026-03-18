package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.commands.SwerveJoystickCmd;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

@TeleOp(name = "Swerve_Control", group = "TeleOp")
public class Swerve_Control extends LinearOpMode {

    private SwerveSubsystem swerveSubsystem;

    private GamepadEx driverGamepad;
    private boolean fieldOriented = true;
    private boolean lastA = false;
    private boolean lastLB = false;

    // 用於整合搖桿成目標位置 (綠點)
    private double targetX = 0;
    private double targetY = 0;
    private double targetHeadingDeg = 0;
    private double lastTime = 0;

    // 成員變數加
    private double speedMultiplier   = 1.0;  // 直走倍率
    private double turningMultiplier = 1.0;  // 轉動倍率
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadLeft  = false;
    private boolean lastDpadRight = false;


    @Override
    public void runOpMode() throws InterruptedException {

        // 傳入 hardwareMap 來初始化 subsystem
        swerveSubsystem = new SwerveSubsystem(hardwareMap);

        // 將標準 gamepad1 包裝成 GamepadEx
        driverGamepad = new GamepadEx(gamepad1);


        // Dashboard + Driver Station telemetry together
        Telemetry dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 創建指令一次即可
        SwerveJoystickCmd joystickCmd = new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverGamepad.getLeftY()  * speedMultiplier,
                () -> -driverGamepad.getLeftX()  * speedMultiplier,
                () -> -driverGamepad.getRightX() * turningMultiplier,
                () -> fieldOriented
        );

        configureButtonBindings();

        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();

        // * 輪子已經在 0° → 現在才重置追蹤基準
        swerveSubsystem.resetAllModuleTracking();

        waitForStart();

        // * 重置航向
        swerveSubsystem.zeroHeading();
        targetHeadingDeg = 0;

        lastTime = getRuntime();

        while (opModeIsActive()) {

            // 刷新按鍵狀態
            driverGamepad.readButtons();

            // Left Bumper：切換 field-oriented / robot-oriented
            boolean lb = driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER);
            if (lb && !lastLB) {
                fieldOriented = !fieldOriented;
            }
            lastLB = lb;

            // A 鍵：重置航向
            boolean a = driverGamepad.getButton(GamepadKeys.Button.A);
            if (a && !lastA) {
                swerveSubsystem.zeroHeading();
                targetHeadingDeg = 0;
            }
            lastA = a;

            // DPAD UP/DOWN：調整直走倍率
            boolean dpadUp   = driverGamepad.getButton(GamepadKeys.Button.DPAD_UP);
            boolean dpadDown = driverGamepad.getButton(GamepadKeys.Button.DPAD_DOWN);
            if (dpadUp   && !lastDpadUp)   speedMultiplier = Math.min(1.0, speedMultiplier + 0.1);
            if (dpadDown && !lastDpadDown) speedMultiplier = Math.max(0.1, speedMultiplier - 0.1);
            lastDpadUp   = dpadUp;
            lastDpadDown = dpadDown;

            // DPAD LEFT/RIGHT：調整轉動倍率
            boolean dpadLeft  = driverGamepad.getButton(GamepadKeys.Button.DPAD_LEFT);
            boolean dpadRight = driverGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT);
            if (dpadRight && !lastDpadRight) turningMultiplier = Math.min(1.0, turningMultiplier + 0.1);
            if (dpadLeft  && !lastDpadLeft)  turningMultiplier = Math.max(0.1, turningMultiplier - 0.1);
            lastDpadLeft  = dpadLeft;
            lastDpadRight = dpadRight;

            // === 用搖桿積分更新目標位置 (綠點) ===
            double now = getRuntime();
            double dt = Math.max(0, now - lastTime);
            lastTime = now;

            // 取得搖桿輸入
            double rawX = driverGamepad.getLeftY();   // 前進為正（推上）
            double rawY = driverGamepad.getLeftX();  // 右為正
            double rawTurn = -driverGamepad.getRightX(); // 順時針為正（推右）

            // deadband
            rawX = Math.abs(rawX) > 0.05 ? rawX : 0;
            rawY = Math.abs(rawY) > 0.05 ? rawY : 0;
            rawTurn = Math.abs(rawTurn) > 0.05 ? rawTurn : 0;


            // 依 DriveConstants 縮放到物理速度
            double xSpeed    = rawX    * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            double ySpeed    = rawY    * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            double turnSpeed = rawTurn * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            // 轉成場地座標速度（若目前是 robot-oriented）
            double headingRad = Math.toRadians(targetHeadingDeg);
            double fieldX = xSpeed;
            double fieldY = ySpeed;
            if (!fieldOriented) {
                double cos = Math.cos(headingRad);
                double sin = Math.sin(headingRad);
                double rx = xSpeed * cos - ySpeed * sin;
                double ry = xSpeed * sin + ySpeed * cos;
                fieldX = rx;
                fieldY = ry;
            }

            // 積分得到目標位置/角度
            targetX += fieldX * dt;
            targetY += fieldY * dt;
            targetHeadingDeg += Math.toDegrees(turnSpeed * dt);

            swerveSubsystem.setTargetPose(targetX, targetY, targetHeadingDeg);

            // 執行指令
            joystickCmd.execute();

            // 更新 subsystem
            swerveSubsystem.periodic();

            // 更新 Dashboard 場地視圖和圖表
            swerveSubsystem.updateTelemetry(dashboardTelemetry);

            // 顯示精簡的必要資訊
            dashboardTelemetry.addData("Mode", fieldOriented ? "Field-Oriented" : "Robot-Oriented");
            dashboardTelemetry.addData("Heading", "%.1f°", swerveSubsystem.getHeading());

            com.arcrobotics.ftclib.geometry.Pose2d currentPose = swerveSubsystem.getPose();
            dashboardTelemetry.addData("Position", "X:%.2f Y:%.2f", currentPose.getX(), currentPose.getY());

            dashboardTelemetry.addData("Target", "X:%.2f Y:%.2f H:%.1f°", targetX, targetY, targetHeadingDeg);
            // 顯示各顆 Drive Motor 的 RPM
            dashboardTelemetry.addData("Drive RPM", "FL:%.0f FR:%.0f BL:%.0f BR:%.0f",
                    swerveSubsystem.getFrontLeft().getDriveRPM(),
                    swerveSubsystem.getFrontRight().getDriveRPM(),
                    swerveSubsystem.getBackLeft().getDriveRPM(),
                    swerveSubsystem.getBackRight().getDriveRPM());

            // 顯示各顆 Turning Motor 的角度 (度數)
            dashboardTelemetry.addData("Turning Angle (deg)", "FL:%.1f° FR:%.1f° BL:%.1f° BR:%.1f°",
                    Math.toDegrees(swerveSubsystem.getFrontLeft().getTurningPosition()),
                    Math.toDegrees(swerveSubsystem.getFrontRight().getTurningPosition()),
                    Math.toDegrees(swerveSubsystem.getBackLeft().getTurningPosition()),
                    Math.toDegrees(swerveSubsystem.getBackRight().getTurningPosition()));

            dashboardTelemetry.addData("Speed",   "%.0f%%  (DPAD↑↓)", speedMultiplier   * 100);
            dashboardTelemetry.addData("Turning", "%.0f%%  (DPAD←→)", turningMultiplier * 100);

            dashboardTelemetry.addData("Status", "Running");
            dashboardTelemetry.update();
        }

        swerveSubsystem.stopModules();
    }

    private void configureButtonBindings(){
        // 已在迴圈內用邊緣觸發處理 A 和 LB
    }

}
