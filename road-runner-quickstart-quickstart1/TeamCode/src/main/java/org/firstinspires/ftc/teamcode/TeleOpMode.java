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
    private double drive, turn, strafe;
    double drive_speed;
    private double drive_speed_max = 0.7;
    private double fr, fl, br, bl, scale;
    private double boxy, boxx;
    private double rightstickx, leftstick;
    private double max;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        while(!isStarted()) {
            init_telemetry();
        }
        waitForStart();
        while(opModeIsActive()){
            init_telemetry();
            teleOpControls();
            chassis_power();
        }
    }

    public void init_hardware(){
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
    }

    public void teleOpControls(){
        drive = -gamepad1.left_stick_y;     //前進
        turn = gamepad1.right_stick_x;      //自旋
        strafe = -gamepad1.left_stick_x;    //平移

        boxy = gamepad1.left_stick_y;
        boxx = gamepad1.left_stick_x;
        rightstickx = Math.abs (gamepad1.right_stick_x);
        leftstick = Math.abs (Math.sqrt (Math.pow (boxy, 2) + Math.pow (boxx, 2)));
    }

    public double scaling_power(double fr, double fl, double br, double bl){
        max = Math.max( Math.max( Math.abs(fr), Math.abs(fl) ), Math.max( Math.abs(br), Math.abs(bl) ));
        if (max <= 1){
            max = 1;
        }
        return max;
    }

    public double drive_speed(double rightstickx, double leftstick){
        if(leftstick < 0.2){
            drive_speed = 0;
        } else if(leftstick < 0.5){
            drive_speed = 0.4;
        } else{
            drive_speed = drive_speed_max;
        }
        if(rightstickx < 0.2){
            drive_speed = 0;
        } else if(rightstickx < 0.5) {
            drive_speed = 0.4;
        } else if(rightstickx < 1){
            drive_speed = drive_speed_max;
        }
        return drive_speed;
    }

    public void chassis_power(){
        fr = (- drive - strafe + turn) * drive_speed(rightstickx, leftstick);
        fl = (- drive + strafe - turn) * drive_speed(rightstickx, leftstick);
        br = (- drive + strafe + turn) * drive_speed(rightstickx, leftstick);
        bl = (- drive - strafe - turn) * drive_speed(rightstickx, leftstick);

        scale = scaling_power(fr, fl, br, bl);
        FR.setPower(fr / scale);
        FL.setPower(fl / scale);
        BR.setPower(br / scale);
        BL.setPower(bl / scale);
    }

    public void init_telemetry(){
        telemetry.addData("FL", FL.getCurrentPosition());
        telemetry.addData("BL", BL.getCurrentPosition());
        telemetry.addData("FR", FR.getCurrentPosition());
        telemetry.addData("BR", BR.getCurrentPosition());
        telemetry.update();
    }
}