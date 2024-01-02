package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {

    private DcMotor FR, FL, BR, BL;
    private double drive_speed = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        while(!isStarted()) {
            init_telemetry();
        }
        waitForStart();
        while(opModeIsActive()){
            init_telemetry();

            double drive = -gamepad1.left_stick_y;     //前進
            double strafe = -gamepad1.left_stick_x;    //平移
            double turn = gamepad1.right_stick_x;      //自旋

            double fr = (-drive - strafe + turn) * drive_speed;
            double fl = (-drive + strafe - turn) * drive_speed;
            double br = (-drive + strafe + turn) * drive_speed;
            double bl = (-drive - strafe - turn) * drive_speed;
            double scale = scaling_power(fr, fl, br, bl);

            FR.setPower(fr / scale);
            FL.setPower(fl / scale);
            BR.setPower(br / scale);
            BL.setPower(bl / scale);
        }
    }

    public void init_hardware(){
        FR=hardwareMap.get(DcMotor.class, "FR");
        FL=hardwareMap.get(DcMotor.class, "FL");
        BR=hardwareMap.get(DcMotor.class, "BR");
        BL=hardwareMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setZeroPowerBehavior(BRAKE);
        BL.setZeroPowerBehavior(BRAKE);
        FR.setZeroPowerBehavior(BRAKE);
        BR.setZeroPowerBehavior(BRAKE);
    }

    public double scaling_power(double fr, double fl, double br, double bl){
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if (max <= 1){
            max = 1;
        }
        return max;
    }


    public void init_telemetry(){
        telemetry.addData("FL", FL.getCurrentPosition());
        telemetry.addData("BL", BL.getCurrentPosition());
        telemetry.addData("FR", FR.getCurrentPosition());
        telemetry.addData("BR", BR.getCurrentPosition());
        telemetry.addData("left_stick_x", -gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", -gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.update();
    }
}