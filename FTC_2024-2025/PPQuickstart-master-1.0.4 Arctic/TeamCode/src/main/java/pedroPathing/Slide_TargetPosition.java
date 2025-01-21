package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

@TeleOp(name = "Slide_Power")
public class Slide_TargetPosition extends LinearOpMode {
    private DcMotorEx SlideLeftFront, SlideLeftBack, SlideRightFront, SlideRightBack;
    private double slide_power = 0.4;
    private int SlidePosition = 0;
    private int MaximumPosition = 1000;
    private int MinimumPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {

            // * Slide
            // * Version II
            if(gamepad1.right_bumper && SlideCurrentPosition() < MaximumPosition) {
                SlidePosition += 1;
            } else if(gamepad1.left_bumper && SlideCurrentPosition() > MinimumPosition) {
                SlidePosition -= 1;
            }
            setSlidePosition(SlidePosition, slide_power);

            telemetry.addLine("Right Bumper: Slide Up");
            telemetry.addLine("Left Bumper: Slide Down");
            telemetry.addData("SlidePosition", SlidePosition);
            telemetry .addData("slide_power: ", slide_power);
            telemetry.addData("Rev Encoder Position", SlideCurrentPosition());
            telemetry.addData("Maximum Position", MaximumPosition);
            telemetry.addData("Minimum Position", MinimumPosition);
            telemetry.update();

        }
    }
    public void init_hardware() {
        // * Slide
        SlideLeftFront = hardwareMap.get(DcMotorEx.class, "SlideLeftFront");
        SlideLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SlideLeftBack = hardwareMap.get(DcMotorEx.class, "SlideLeftBack");
        SlideLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        SlideRightFront = hardwareMap.get(DcMotorEx.class, "SlideRightFront");
        SlideRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        SlideRightBack = hardwareMap.get(DcMotorEx.class, "SlideRightBack");
        SlideRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setSlidePosition(SlidePosition, slide_power);

    }

    public void setSlidePosition(int position, double slide_power) {
        SlideLeftFront.setTargetPosition(position);
        SlideLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideLeftFront.setPower(slide_power);

        SlideLeftBack.setTargetPosition(position);
        SlideLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideLeftBack.setPower(slide_power);

        SlideRightFront.setTargetPosition(position);
        SlideRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideRightFront.setPower(slide_power);

        SlideRightBack.setTargetPosition(position);
        SlideRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideRightBack.setPower(slide_power);
    }

    public double SlideCurrentPosition() {
        return SlideRightBack.getCurrentPosition();
    }
}
