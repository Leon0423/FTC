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
    private double intake_maxPosition = 0.4;
    private double intake_minPosition = 0.0;
    private double intake_pos = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * Chassis
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

            // * Horizon Slide
            // ? 測試看看Horizon Slide and intake能不能成功運轉（因為太常換班的）
            // ! gamepad "2" is the second controller
            // * 用順邊測測看極限值
            HorizonSlide_position = Math.min(HorizonSlide_maxPosition, Math.max(HorizonSlide_minPosition, HorizonSlide_position));

            HSR.setPosition(HorizonSlide_position += (-gamepad2.left_stick_y) * 0.001);
            HSL.setPosition(HorizonSlide_position += (-gamepad2.left_stick_y) * 0.001);

            // * intake
            intake_pos = Math.min(intake_maxPosition, Math.max(intake_minPosition, intake_pos));
            intake_pos += -gamepad2.right_stick_y * 0.01;

            intakeRight.setPosition(intake_pos);
            intakeLeft.setPosition(intake_pos);

            if(gamepad2.x) {
                intake_claw.setPosition(intake_claw.getPosition() + 0.001);
            }
            if(gamepad2.y) {
                intake_claw.setPosition(intake_claw.getPosition() - 0.001);
            }

            // * telemetry
            // * chassis
            telemetry.addData("center", FR.getCurrentPosition());
            telemetry.addData("left", FL.getCurrentPosition());
            telemetry.addData("right", BL.getCurrentPosition());

            // * Horizon Slide
            telemetry.addLine("HorizonSlide: left_stick_y");
            telemetry.addData("HSR", HSR.getPosition());
            telemetry.addData("HSL", HSL.getPosition());
            telemetry.addData("HorizonSlide_position", HorizonSlide_position);

            // * intake
            telemetry.addLine("intakeRight: right_stick_y");
            telemetry.addData("intakeRight", intakeRight.getPosition());
            telemetry.addData("intakeLeft", intakeLeft.getPosition());
            telemetry.addData("intake_pos", intake_pos);

            telemetry.addLine();
            telemetry.addLine("intake_claw: X/Y");
            telemetry.addData("intake_claw", intake_claw.getPosition());


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
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // * Horizon Slide
        HSL = hardwareMap.get(Servo.class, "servoLeft");
        HSR = hardwareMap.get(Servo.class, "servoRight");
        HSR.setDirection(Servo.Direction.REVERSE);

        // * intake
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intake_claw = hardwareMap.get(Servo.class, "intake_claw");
        intakeRight.setDirection(Servo.Direction.REVERSE);
        intakeRight.setPosition(intake_pos);
        intakeLeft.setPosition(intake_pos);

    }
    public double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if(max <= 1) {
            max = 1;
        }
        return max;
    }
}
