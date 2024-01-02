package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MathCaculate")
public class MathCaculate extends LinearOpMode {

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

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin, cos, max;

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            double fl = power * cos / max + turn;
            double fr = power * cos / max - turn;
            double bl = power * cos / max + turn;
            double br = power * cos / max - turn;

            if ((power + Math.abs(turn)) > 1) {
                fl /= power + Math.abs(turn);
                fr /= power + Math.abs(turn);
                bl /= power + Math.abs(turn);
                br /= power + Math.abs(turn);
            }

            FL.setPower(fl *drive_speed);
            FR.setPower(fr * drive_speed);
            BL.setPower(bl * drive_speed);
            BR.setPower(br * drive_speed);
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

    public void init_telemetry(){
        telemetry.addData("FL", FL.getCurrentPosition());
        telemetry.addData("BL", BL.getCurrentPosition());
        telemetry.addData("FR", FR.getCurrentPosition());
        telemetry.addData("BR", BR.getCurrentPosition());
        telemetry.update();
    }
}