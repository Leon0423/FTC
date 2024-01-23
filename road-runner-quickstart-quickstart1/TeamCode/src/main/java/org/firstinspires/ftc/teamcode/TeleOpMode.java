package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {

    private DcMotorEx FR, FL, BR, BL;
    private DcMotorEx Slide_left, Slide_right;
    private DcMotorEx intake;
    private double drive_speed = 1 ;
    private double Slide_speed = 0.1;

    private int motor_pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        while(!isStarted()) {
            init_telemetry();
        }
        waitForStart();
        while(opModeIsActive()){
            init_telemetry();

            //drive_part
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

            //Slide_part
            int per_updown;

            if (gamepad1.right_trigger > 0.5 || gamepad1.left_trigger > 0.5){
                per_updown = 10;
            } else {
                per_updown = 5;
            }

            if(gamepad1.right_trigger > 0 && motor_pos < 1000) {
                motor_pos += per_updown;
            }
            else if(gamepad1.left_trigger > 0 && motor_pos > 0) {
                motor_pos -= per_updown;
            }
            Slide_right.setTargetPosition(motor_pos);
            Slide_left.setTargetPosition(motor_pos);

            //intake_part
            if (gamepad1.a){
                intake.setPower(1);
            }else {
                intake.setPower(0);
            }
        }
    }

    public void init_hardware(){
        //chassis_Base
        FR=hardwareMap.get(DcMotorEx.class, "FR");
        FL=hardwareMap.get(DcMotorEx.class, "FL");
        BR=hardwareMap.get(DcMotorEx.class, "BR");
        BL=hardwareMap.get(DcMotorEx.class, "BL");

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

        //slide_Base
        Slide_left=hardwareMap.get(DcMotorEx.class, "SL");
        Slide_right=hardwareMap.get(DcMotorEx.class, "SR");

        Slide_right.setDirection(DcMotorSimple.Direction.REVERSE);

        Slide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide_right.setZeroPowerBehavior(BRAKE);
        Slide_left.setZeroPowerBehavior(BRAKE);

        Slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide_right.setTargetPosition(0);
        Slide_left.setTargetPosition(0);

        Slide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slide_right.setTargetPosition(0);
        Slide_left.setTargetPosition(0);

        Slide_right.setPower(Slide_speed);
        Slide_left.setPower(Slide_speed);

        //intake_Base
        intake=hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(FLOAT);
    }

    public double scaling_power(double fr, double fl, double br, double bl){
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if (max <= 1){
            max = 1;
        }
        return max;
    }

    public void init_telemetry(){
        telemetry.addData("drive_speed", drive_speed);
        telemetry.addData("Slide_speed", Slide_speed);

        telemetry.addData("left_trigger", gamepad1.left_trigger);
        telemetry.addData("right_trigger", gamepad1.right_trigger);

        telemetry.addData("motor_pos", motor_pos);

        telemetry.addData("Slide_right", Slide_right.getCurrentPosition());
        telemetry.addData("Slide_left", Slide_left.getCurrentPosition());

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