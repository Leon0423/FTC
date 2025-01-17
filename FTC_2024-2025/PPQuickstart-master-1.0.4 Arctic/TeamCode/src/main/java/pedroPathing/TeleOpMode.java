package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    private DcMotorEx FR, FL, BR, BL;
    double drive, turn, strafe;
    double fr, fl, br, bl, scale;
    private DcMotorEx SlideLeftFront, SlideLeftBack, SlideRightFront, SlideRightBack;
    private Servo OutputClaw, OutputCenter, OutputLeft, OutputRight;
    private double slide_power = 0.4;

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

            // * Slide
            if(gamepad1.a) {
                // * Slide up
                SlideLeftFront.setPower(slide_power);
                SlideLeftBack.setPower(slide_power);
                SlideRightFront.setPower(slide_power);
                SlideRightBack.setPower(slide_power);
            } else if (gamepad1.b) {
                // * Slide down
                SlideLeftFront.setPower(-slide_power);
                SlideLeftBack.setPower(-slide_power);
                SlideRightFront.setPower(-slide_power);
                SlideRightBack.setPower(-slide_power);
            } else {
                SlideLeftFront.setPower(0);
                SlideLeftBack.setPower(0);
                SlideRightFront.setPower(0);
                SlideRightBack.setPower(0);
            }

            // * Output
            if(gamepad1.dpad_up) {
                OutputLeft.setPosition(OutputLeft.getPosition() + 0.01);
                OutputRight.setPosition(OutputRight.getPosition() + 0.01);
            } else if(gamepad1.dpad_down) {
                OutputLeft.setPosition(OutputLeft.getPosition() - 0.01);
                OutputRight.setPosition(OutputRight.getPosition() - 0.01);
            }

            if(gamepad1.dpad_left) {
                OutputCenter.setPosition(OutputCenter.getPosition() + 0.01);
            } else if(gamepad1.dpad_right) {
                OutputCenter.setPosition(OutputCenter.getPosition() - 0.01);
            }

            if(gamepad1.x) {
                OutputClaw.setPosition(OutputClaw.getPosition() + 0.01);
            } else if(gamepad1.y) {
                OutputClaw.setPosition(OutputClaw.getPosition() - 0.01);
            }
            // * Chassis
            telemetry.addData("slide encoder", BR.getCurrentPosition());
            telemetry.addData("left", FL.getCurrentPosition());
            telemetry.addData("right", BL.getCurrentPosition());

            // * Slide
            telemetry.addLine("A: Slide Up");
            telemetry.addLine("B: Slide Down");

            // * Output
            telemetry.addLine("Dpad Up/Down: OutputLeft, OutputRight");
            telemetry.addData("OutputLeft", OutputLeft.getPosition());
            telemetry.addData("OutputRight", OutputRight.getPosition());

            telemetry.addLine("Dpad Left/Right: OutputCenter");
            telemetry.addData("OutputCenter", OutputCenter.getPosition());

            telemetry.addLine("X/Y: OutputClaw");
            telemetry.addData("OutputClaw", OutputClaw.getPosition());

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

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // * Slide
        SlideLeftFront = hardwareMap.get(DcMotorEx.class, "SlideLeftFront");
        SlideLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideLeftBack = hardwareMap.get(DcMotorEx.class, "SlideLeftBack");
        SlideLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideRightFront = hardwareMap.get(DcMotorEx.class, "SlideRightFront");
        SlideRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideRightBack = hardwareMap.get(DcMotorEx.class, "SlideRightBack");
        SlideRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // * Output
        OutputClaw = hardwareMap.get(Servo.class, "OutputClaw");
        OutputClaw.setPosition(0);

        OutputCenter = hardwareMap.get(Servo.class, "OutputCenter");
        OutputCenter.setPosition(0);

        OutputLeft = hardwareMap.get(Servo.class, "OutputLeft");
        OutputLeft.setDirection(Servo.Direction.FORWARD);
        OutputLeft.setPosition(0);

        OutputRight = hardwareMap.get(Servo.class, "OutputRight");
        OutputRight.setDirection(Servo.Direction.REVERSE);
        OutputRight.setPosition(0);

    }
    public double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if(max <= 1) {
            max = 1;
        }
        return max;
    }
}
