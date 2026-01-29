package org.firstinspires.ftc.teamcode.flyWheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PIDFControl", group = "Test")
public class PIDFControl extends LinearOpMode {
    public DcMotorEx flywheelMotor;

    // RPM 設定
    public static final double TICKS_PER_REV = 28.0; // 根據你的馬達編碼器調整

    public double highRPM = 4671;
    public double lowRPM = highRPM - 500;

    double curTargetRPM = highRPM;

    double F = 0;
    double P = 0;
    double I = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    // RPM 與 Velocity (ticks/s) 轉換
    private double rpmToVelocity(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private double velocityToRpm(double velocity) {
        return velocity * 60.0 / TICKS_PER_REV;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("init complete");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            if(gamepad1.yWasPressed()){
                if(curTargetRPM == highRPM){
                    curTargetRPM = lowRPM;
                }else{
                    curTargetRPM = highRPM;
                }
            }

            if (gamepad1.bWasPressed()){
                stepIndex = (stepIndex + 1) % stepSizes.length;
                telemetry.addData("Step Size", stepSizes[stepIndex]);
                telemetry.update();
            }

            if (gamepad1.dpadLeftWasPressed()){
                F -= stepSizes[stepIndex];
            }

            if (gamepad1.dpadRightWasPressed()){
                F += stepSizes[stepIndex];
            }

            if (gamepad1.dpadUpWasPressed()){
                P += stepSizes[stepIndex];
            }

            if (gamepad1.dpadDownWasPressed()){
                P -= stepSizes[stepIndex];
            }

            // 加入 I 調整按鈕
            if (gamepad1.leftBumperWasPressed()) {
                I += stepSizes[stepIndex];
            }
            if (gamepad1.rightBumperWasPressed()) {
                I -= stepSizes[stepIndex];
            }

            // set new PIDF coefficients
            pidfCoefficients = new PIDFCoefficients(P, I, 0, F);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // 將 RPM 轉換為 velocity 並設定
            double targetVelocity = rpmToVelocity(curTargetRPM);
            flywheelMotor.setVelocity(targetVelocity);

            double curVelocity = flywheelMotor.getVelocity();
            double curRPM = velocityToRpm(curVelocity);
            double errorRPM = curTargetRPM - curRPM;

            telemetry.addLine("========== RPM 資訊 ==========");
            telemetry.addData("目標 RPM", "%.0f", curTargetRPM);
            telemetry.addData("現在 RPM", "%.2f", curRPM);
            telemetry.addData("Error", "%.2f RPM", errorRPM);
            telemetry.addLine();
            telemetry.addLine("========== PIDF 調整 ==========");
            telemetry.addData("Tuning P", "%.4f (D-Pad Up/Down)", P);
            telemetry.addData("Tuning F", "%.4f (D-Pad Left/Right)", F);
            telemetry.addData("Tuning I", "%.4f (Bumpers)", I);
            telemetry.addData("調整大小", "%.4f (B Button)", stepSizes[stepIndex]);
            telemetry.addLine();
            telemetry.addData("目標 Velocity", "%.2f ticks/s", targetVelocity);
            telemetry.addLine("提示: Y=切換RPM | B=改變調整大小");
            telemetry.update();
        }
    }
}
