package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode_IntakeMode")
public class TeleOpMode_IntakeMode extends LinearOpMode {
    private Servo horizonSlideLeft, horizonSlideRight;
    private Servo intakeLeft, intakeRight;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {

            // * intake
            if(gamepad1.y){
                horizonSlideLeft.setPosition(horizonSlideLeft.getPosition() + 0.01);
                horizonSlideRight.setPosition(horizonSlideRight.getPosition() + 0.01);
            } else if (gamepad1.a) {
                horizonSlideLeft.setPosition(horizonSlideLeft.getPosition() - 0.01);
                horizonSlideRight.setPosition(horizonSlideRight.getPosition() - 0.01);
            }

            if(gamepad1.b) {
                horizonSlideLeft.setPosition(0.3);
                horizonSlideRight.setPosition(0.3);
            }



            // * intake
            telemetry.addLine("Y/A: Intake Forward / Backward");
            telemetry.addData("IntakeRight", horizonSlideRight.getPosition());
            telemetry.addData("IntakeLeft", horizonSlideLeft.getPosition());
            telemetry.update();

        }
    }
    public void init_hardware() {

        // * intake
        horizonSlideLeft = hardwareMap.get(Servo.class, "HorizonLeft");
        horizonSlideRight = hardwareMap.get(Servo.class, "HorizonRight");
        horizonSlideRight.setDirection(Servo.Direction.REVERSE);
        horizonSlideRight.setPosition(0);
        horizonSlideLeft.setPosition(0);

        intakeLeft = hardwareMap.get(Servo.class, "IntakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "IntakeRight");
        intakeRight.setDirection(Servo.Direction.REVERSE);
        intakeRight.setPosition(0);
        intakeLeft.setPosition(0);



    }
}
