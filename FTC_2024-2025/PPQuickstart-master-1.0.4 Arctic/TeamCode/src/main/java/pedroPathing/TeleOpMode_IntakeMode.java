package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode_IntakeMode")
public class TeleOpMode_IntakeMode extends LinearOpMode {
    private Servo IntakeLeft, IntakeRight;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while(opModeIsActive()) {

            // * intake
            if(gamepad1.y){
                IntakeLeft.setPosition(IntakeLeft.getPosition() + 0.01);
                IntakeRight.setPosition(IntakeRight.getPosition() + 0.01);
            } else if (gamepad1.a) {
                IntakeLeft.setPosition(IntakeLeft.getPosition() - 0.01);
                IntakeRight.setPosition(IntakeRight.getPosition() - 0.01);
            }

            if(gamepad1.b) {
                IntakeLeft.setPosition(0.3);
                IntakeRight.setPosition(0.3);
            }
            if(gamepad1.x){
                IntakeLeft.setPosition(-0.3);
                IntakeRight.setPosition(-0.3);
            }


            // * intake
            telemetry.addLine("Y/A: Intake Forward / Backward");
            telemetry.addData("IntakeRight", IntakeRight.getPosition());
            telemetry.addData("IntakeLeft", IntakeLeft.getPosition());
            telemetry.update();

        }
    }
    public void init_hardware() {

        // * intake
        IntakeLeft = hardwareMap.get(Servo.class, "HorizonLeft");
        IntakeRight = hardwareMap.get(Servo.class, "HorizonRight");
        IntakeRight.setDirection(Servo.Direction.REVERSE);
        IntakeRight.setPosition(0);
        IntakeLeft.setPosition(0);


    }
}
