package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    private DcMotorEx FR, FL, BR, BL;
    double drive, turn, strafe;
    double fr, fl, br, bl, scale;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            drive = -gamepad1.left_stick_y;     // * 前進
            turn = gamepad1.right_stick_x;      // * 自旋
            strafe = gamepad1.left_stick_x;    // * 平移

            fr = drive - turn - strafe;
            fl = drive + turn + strafe;
            br = drive - turn + strafe;
            bl = drive + turn - strafe;

            scale = scaling_power(fr, fl, br, bl); // * 取得最大值

            FR.setPower(fr/scale);
            FL.setPower(fl/scale);
            BR.setPower(br/scale);
            BL.setPower(bl/scale);

            //TODO: add negative power to the motors
            telemetry.addData("center", FR.getCurrentPosition());
            telemetry.addData("left", FL.getCurrentPosition());
            telemetry.addData("right", BL.getCurrentPosition());

            telemetry.update();
        }
    }
    public void init_hardware() {
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



    }
    public double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if(max <= 1) {
            max = 1;
        }
        return max;
    }
}
