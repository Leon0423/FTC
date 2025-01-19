package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode_IntakeMode")
public class TeleOpMode_IntakeMode extends LinearOpMode {
    private CRServo HSRight, HSLeft;
    private Servo HSArmLeft, HSArmRight, IntakeClaw;
    AnalogInput HSLeftEncoder, HSRightEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {

            // * Horizon Slide
            double speed = 0.5;
            HSRight.setPower(-gamepad1.right_stick_y * speed); // ! speed need 緩衝(Check the Voltage)
            HSLeft.setPower(-gamepad1.right_stick_y * speed);

            // * Horizon Arm

            HSArmRight.setPosition(HSArmRight.getPosition() + (-gamepad1.left_stick_y) * 0.01);
            HSArmLeft.setPosition(HSArmLeft.getPosition() + (-gamepad1.left_stick_y) * 0.01);

            if(gamepad1.a){
                HSArmLeft.setPosition(HSArmLeft.getPosition() + 0.001);
            } else if (gamepad1.b) {
                HSArmLeft.setPosition(HSArmLeft.getPosition() - 0.001);
            }

            if(gamepad1.x){
                HSArmRight.setPosition(HSArmRight.getPosition() + 0.001);
            } else if (gamepad1.y) {
                HSArmRight.setPosition(HSArmRight.getPosition() - 0.001);
            }

            if(gamepad1.dpad_up) {
                transferSample1Position();
            } else if (gamepad1.dpad_down) {
                transferSample2Position();
            }


            // * intake

            telemetry.addData("HSRRightEncoderVoltage", HSRightEncoder.getVoltage());

            telemetry.addData("HSRightPower", HSRight.getPower());
            telemetry.addData("HSLeftPower", HSLeft.getPower());

            telemetry.addData("HSArmLeftPosition", HSArmLeft.getPosition());
            telemetry.addData("HSArmRightPosition", HSArmRight.getPosition());

            telemetry.addData("HSRightEncoderPosition", HSRightEncoder.getVoltage() / 3.3 * 360);


            telemetry.update();



        }
    }
    public void init_hardware() {

        // * intake
        HSRight = hardwareMap.get(CRServo.class, "HSRight");
        HSLeft = hardwareMap.get(CRServo.class, "HSLeft");
        HSRight.setDirection(CRServo.Direction.REVERSE);
        HSLeft.setDirection(CRServo.Direction.FORWARD);
        HSRight.setPower(0);
        HSLeft.setPower(0);

        HSArmLeft = hardwareMap.get(Servo.class, "HSArmLeft");
        HSArmRight = hardwareMap.get(Servo.class, "HSArmRight");
        HSArmRight.setDirection(Servo.Direction.REVERSE);
        HSArmLeft.setPosition(0);
        HSArmRight.setPosition(0);

        // * analog input
        HSLeftEncoder = hardwareMap.get(AnalogInput.class, "HSLeftEncoder");
        HSRightEncoder = hardwareMap.get(AnalogInput.class, "HSRightEncoder");

        // * Intake Claw
        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");
    }

    public void transferSample1Position() {
        HSArmLeft.setPosition(0.23);
        HSArmRight.setPosition(0.23);
    }

    public void transferSample2Position() {
        HSArmLeft.setPosition(0.55);
        HSArmRight.setPosition(0.17);
    }
}
