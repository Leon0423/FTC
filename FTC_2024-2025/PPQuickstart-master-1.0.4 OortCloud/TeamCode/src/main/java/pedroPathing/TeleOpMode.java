package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    // * chassis
    private DcMotorEx FR, FL, BR, BL;
    private double drive, turn, strafe;
    private double fr, fl, br, bl, scale;

    // * Horizon Slide
    private Servo HSR, HSL;
    private double HorizonSlide_maxPosition = 0.43;
    private double HorizonSlide_minPosition = 0.0;
    double HorizonSlide_position = 0;

    // * intake
    private Servo intakeLeft, intakeRight, intake_claw;
    private double intake_maxPosition = 0.75;
    private double intake_minPosition = 0.0;
    private double intake_pos = 0.0;

    double intake_ClawOpenPosition = 0.4;
    double intake_ClawClosePosition = 0.6;

    // * Slide
    DcMotorEx SlideLeft, SlideRight;
    private int SlideMaximumPosition = 3800;
    private int SlideMinimumPosition = 0;
    private int SlidePosition = 0;
    private double SlidePower = 0.6;

    // * Arm
    private Servo ArmLeft, ArmRight;
    private Servo OutputClaw;
    private double ArmInitialPosition = 0.0;
    private double Arm_Position = 0;

    // * OutputClaw
    private final double Output_ClawClosePosition = 0.65;
    private final double Output_ClawOpenPosition = 0.3;


    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * Chassis
            // * --------------------------------------------------- //
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


            // * Horizon Slide: gamepad2.left_stick_y
            // * Horizon Slide position: 0.0 ~ 0.43
            // * --------------------------------------------------- //
            HorizonSlide_position = Math.min(HorizonSlide_maxPosition, Math.max(HorizonSlide_minPosition, HorizonSlide_position));
            HorizonSlide_position += -0.005 * gamepad2.left_stick_y;

            HSR.setPosition(HorizonSlide_position);
            HSL.setPosition(HorizonSlide_position);



            // * intake: gamepad2.right_stick_y
            // * intake position: 0.0 ~ 0.8
            // * --------------------------------------------------- //
            intake_pos = Math.min(intake_maxPosition, Math.max(intake_minPosition, intake_pos));
            intake_pos += (-gamepad2.right_stick_y) * 0.005;

            intakeRight.setPosition(intake_pos);
            intakeLeft.setPosition(intake_pos);


            // * intake_claw: gamepad2.dpadright / gamepad2.dpadLeft
            // * --------------------------------------------------- //
            if(gamepad2.dpad_right) {
                intake_claw.setPosition(intake_ClawOpenPosition);
            }
            if(gamepad2.dpad_left) {
                intake_claw.setPosition(intake_ClawClosePosition);
            }

            // * Slide: gamepad2.a / gamepad2.b
            // * --------------------------------------------------- //
            if(gamepad2.a && SlidePosition < SlideMaximumPosition) {
                SlidePosition += 15;
            } else if (gamepad2.b && SlidePosition > SlideMinimumPosition) {
                SlidePosition -= 15;
            }

            SlideLeft.setTargetPosition(SlidePosition);
            SlideRight.setTargetPosition(SlidePosition);

            // * Arm: gamepad2.x / gamepad2.y
            Arm_Position = Math.min(0.8, Math.max(0, Arm_Position));
            if(gamepad2.x) {
                Arm_Position += 0.005;
            }
            if(gamepad2.y) {
                Arm_Position -= 0.005;
            }

            ArmLeft.setPosition(Arm_Position);
            ArmRight.setPosition(Arm_Position);

            // * OutputClaw: gamepad2.dpad_up / gamepad2.dpad_down
            if(gamepad2.dpad_up) {
                OutputClaw.setPosition(Output_ClawOpenPosition);
            }
            if(gamepad2.dpad_down) {
                OutputClaw.setPosition(Output_ClawClosePosition);
            }

            // * telemetry
            // * chassis

            // * Horizon Slide

            telemetry.addData("HSR", HSR.getPosition());
            telemetry.addData("HSL", HSL.getPosition());
            telemetry.addData("HorizonSlide_position", HorizonSlide_position);
            telemetry.addLine(" ");

            // * intake
            telemetry.addData("intakeRight", intakeRight.getPosition());
            telemetry.addData("intakeLeft", intakeLeft.getPosition());
            telemetry.addData("intake_pos", intake_pos);
            telemetry.addLine(" ");

            telemetry.addLine("intake_claw: dpad_right / dpad_left");
            telemetry.addData("intake_claw", intake_claw.getPosition());
            telemetry.addLine(" ");

            // * Slide
            telemetry.addData("SlidePosition", SlidePosition);
            telemetry.addData("SlidePower", SlidePower);
            telemetry.addData("Left Target Position", SlideLeft.getTargetPosition());
            telemetry.addData("Right Target Position", SlideRight.getTargetPosition());
            telemetry.addLine(" ");

            // * Arm
            telemetry.addData("ArmLeftPosition", ArmLeft.getPosition());
            telemetry.addData("ArmRightPosition", ArmRight.getPosition());
            telemetry.addLine(" ");

            //* OutputClaw
            telemetry.addData("OutputClawPosition", OutputClaw.getPosition());
            telemetry.addLine(" ");

            telemetry.update();
        }
    }
    public void init_hardware() {
        // * chassis
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // * Horizon Slide
        HSL = hardwareMap.get(Servo.class, "servoLeft");
        HSR = hardwareMap.get(Servo.class, "servoRight");
        HSR.setDirection(Servo.Direction.REVERSE);

        HSR.setPosition(HorizonSlide_position);
        HSL.setPosition(HorizonSlide_position);

        // * intake
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intake_claw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeRight.setDirection(Servo.Direction.REVERSE);
        intakeRight.setPosition(intake_pos);
        intakeLeft.setPosition(intake_pos);

        intake_claw.setPosition(intake_ClawClosePosition);

        // * Slide
        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        SlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SlideLeft.setTargetPosition(SlidePosition);
        SlideRight.setTargetPosition(SlidePosition);
        SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideLeft.setTargetPosition(SlidePosition);
        SlideRight.setTargetPosition(SlidePosition);
        SlideLeft.setPower(SlidePower);
        SlideRight.setPower(SlidePower);

        // * Arm
        ArmLeft = hardwareMap.get(Servo.class, "ArmLeft");
        ArmRight = hardwareMap.get(Servo.class, "ArmRight");
        ArmRight.setDirection(Servo.Direction.REVERSE);
        ArmLeft.setPosition(ArmInitialPosition);
        ArmRight.setPosition(ArmInitialPosition);

        // * OutputClaw
        OutputClaw = hardwareMap.get(Servo.class, "OutputClaw");
        OutputClaw.setPosition(Output_ClawClosePosition);

    }
    public double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if(max <= 1) {
            max = 1;
        }
        return max;
    }
}
