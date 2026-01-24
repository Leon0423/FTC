package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PIDFControl", group = "Linear Opmode")
public class PIDFControl extends LinearOpMode {
    public DcMotorEx flywheelMotor;

    public double highVelocity = 2000;
    public double lowVelocity = highVelocity - 200;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

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
                if(curTargetVelocity == highVelocity){
                    curTargetVelocity = lowVelocity;
                }else{
                    curTargetVelocity = highVelocity;
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


            // set new PIDF coefficients
            pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


            // set velocity
            flywheelMotor.setVelocity(curTargetVelocity);

            double curVelocity = flywheelMotor.getVelocity();
            double error = curTargetVelocity - curVelocity;

            telemetry.addLine("========== 速度資訊 ==========");
            telemetry.addData("目標速度", "%.0f", curTargetVelocity);
            telemetry.addData("當前速度", "%.2f", curVelocity);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addLine();
            telemetry.addLine("========== PIDF 調整 ==========");
            telemetry.addData("Tuning P", "%.4f (D-Pad Up/Down)", P);
            telemetry.addData("Tuning F", "%.4f (D-Pad Left/Right)", F);
            telemetry.addData("調整大小", "%.4f (B Button)", stepSizes[stepIndex]);
            telemetry.addLine();
            telemetry.addLine("提示: Y=切換速度 | B=改變更改大小");
            telemetry.update();
        }
    }
}
