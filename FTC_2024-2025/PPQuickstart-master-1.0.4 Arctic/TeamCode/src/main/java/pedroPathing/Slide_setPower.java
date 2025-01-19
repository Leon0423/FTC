package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.CommandPackage.Subsystems.OutputSubsystem;

@TeleOp(name = "Slide_setPower")
public class Slide_setPower extends LinearOpMode {
    private DcMotorEx SlideLeftFront, SlideLeftBack, SlideRightFront, SlideRightBack;
    private double slide_power = 0.4;
    private int MaximumPosition = 1000;
    private int MinimumPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {
            // * Slide
            // * Version I

            if(SlideCurrentPosition() < MaximumPosition && SlideCurrentPosition() > MinimumPosition) {
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
            } else if (SlideCurrentPosition() > MaximumPosition) {
                SlideLeftFront.setPower(-slide_power);
                SlideLeftBack.setPower(-slide_power);
                SlideRightFront.setPower(-slide_power);
                SlideRightBack.setPower(-slide_power);
            } else if (SlideCurrentPosition() < MinimumPosition) {
                SlideLeftFront.setPower(slide_power);
                SlideLeftBack.setPower(slide_power);
                SlideRightFront.setPower(slide_power);
                SlideRightBack.setPower(slide_power);
            }


            // * Slide
            telemetry.addLine("Right Bumper: Slide Up");
            telemetry.addLine("Left Bumper: Slide Down");
            telemetry.addData("slide_power: ", slide_power);
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

    }

    public double SlideCurrentPosition() {
        return SlideRightBack.getCurrentPosition();
    }
}
