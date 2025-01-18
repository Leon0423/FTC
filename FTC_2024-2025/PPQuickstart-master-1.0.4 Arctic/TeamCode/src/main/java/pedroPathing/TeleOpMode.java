package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    private DcMotorEx FR, FL, BR, BL;
    double drive, turn, strafe;
    double fr, fl, br, bl, scale;
    private DcMotorEx SlideLeftFront, SlideLeftBack, SlideRightFront, SlideRightBack;
    private Servo IntakeLeft, IntakeRight;
    private
    OutputSubsystem outputSubsystem;
    double slide_power = 0.4;

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

            if(gamepad1.right_bumper) {
                // * Slide up
                SlideLeftFront.setPower(slide_power);
                SlideLeftBack.setPower(slide_power);
                SlideRightFront.setPower(slide_power);
                SlideRightBack.setPower(slide_power);
            } else if (gamepad1.left_bumper) {
                // * Slide down
                SlideLeftFront.setPower(-slide_power);
                SlideLeftBack.setPower(-slide_power);
                SlideRightFront.setPower(-slide_power);
                SlideRightBack.setPower(-slide_power);
            } else {
                SlideLeftFront.setPower(0.00001);
                SlideLeftBack.setPower(0.00001);
                SlideRightFront.setPower(0.00001);
                SlideRightBack.setPower(0.00001);
            }

            // * Output
            if(gamepad1.dpad_up) {
                outputSubsystem.outputLeft(outputSubsystem.getOutputLeftCurrentPosition() + 0.01);
                outputSubsystem.outputRight(outputSubsystem.getOutputRightCurrentPosition() + 0.01);
            } else if(gamepad1.dpad_down) {
                outputSubsystem.outputLeft(outputSubsystem.getOutputLeftCurrentPosition() - 0.01);
                outputSubsystem.outputRight(outputSubsystem.getOutputRightCurrentPosition() - 0.01);
            }

            if(gamepad1.dpad_left) {
                outputSubsystem.outputCenter(outputSubsystem.getOutputCenterCurrentPosition() + 0.01);
            } else if(gamepad1.dpad_right) {
                outputSubsystem.outputCenter(outputSubsystem.getOutputCenterCurrentPosition() - 0.01);
            }

            if(gamepad1.x) {
                outputSubsystem.outputClaw(outputSubsystem.getOutputClawCurrentPosition() + 0.01);
            } else if(gamepad1.y) {
                outputSubsystem.outputClaw(outputSubsystem.getOutputClawCurrentPosition() - 0.01);
            }

            // * intake
            if(gamepad1.y){
                IntakeLeft.setPosition(IntakeLeft.getPosition() + 0.01);
                IntakeRight.setPosition(IntakeRight.getPosition() + 0.01);
            } else if (gamepad1.a) {
                IntakeLeft.setPosition(IntakeLeft.getPosition() - 0.01);
                IntakeRight.setPosition(IntakeRight.getPosition() - 0.01);
            }


            // * Chassis
            telemetry.addData("slide encoder", BR.getCurrentPosition());
            telemetry.addData("left", FL.getCurrentPosition());
            telemetry.addData("right", BL.getCurrentPosition());

            // * Slide
            telemetry.addLine("Right Bumper: Slide Up");
            telemetry.addLine("Left Bumper: Slide Down");

            telemetry.addData("SlideCurrentPosition", SlideCurrentPosition());

            // * Output
            telemetry.addLine("Dpad Up / Down: OutputLeft, OutputRight");
            telemetry.addData("OutputLeft", outputSubsystem.getOutputLeftCurrentPosition());
            telemetry.addData("OutputRight", outputSubsystem.getOutputRightCurrentPosition());

            telemetry.addLine("Dpad Left / Right: OutputCenter");
            telemetry.addData("OutputCenter", outputSubsystem.getOutputCenterCurrentPosition());

            telemetry.addLine("X / Y: OutputClaw");
            telemetry.addData("OutputClaw", outputSubsystem.getOutputClawCurrentPosition());

            // * intake
            telemetry.addLine("Y/A: Intake Forward / Backward");
            telemetry.addData("IntakeRight", IntakeRight.getPosition());
            telemetry.addData("IntakeLeft", IntakeLeft.getPosition());
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
        SlideRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // * Output
        outputSubsystem = new OutputSubsystem(hardwareMap);
        outputSubsystem.setDefaultPosition();

        // * intake
        IntakeLeft = hardwareMap.get(Servo.class, "HorizonLeft");
        IntakeRight = hardwareMap.get(Servo.class, "HorizonRight");
        IntakeRight.setDirection(Servo.Direction.REVERSE);
        IntakeRight.setPosition(0);
        IntakeLeft.setPosition(0);


    }
    public double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if(max <= 1) {
            max = 1;
        }
        return max;
    }

    public double SlideCurrentPosition() {
        return SlideRightBack.getCurrentPosition();
    }
}
