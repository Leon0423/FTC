package org.firstinspires.ftc.teamcode.drive.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SwerveDrive")
public class SwerveDrive extends SwerveDrive_Base {
    double drive,Xvalue,servoposition,strafe=0.2,straight=0.5;
    double power=1;

    @Override
    public void runOpMode() throws InterruptedException{
        init_hardware();
        frServo.setPosition(straight);
        flServo.setPosition(straight);
        brServo.setPosition(straight);
        blServo.setPosition(straight);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("servoposition",servoposition);
            telemetry.addData("frservoposition",servoposition);
            telemetry.addData("flservoposition",servoposition);
            telemetry.addData("brservoposition",servoposition);
            telemetry.addData("blservoposition",servoposition);
            telemetry.update();
            drive=gamepad1.left_stick_y;
            FR.setPower(drive*power);
            FL.setPower(drive*power);
            BR.setPower(drive*power);
            BL.setPower(drive*power);

            //Xvalue= -gamepad1.right_stick_x;
            //servoposition=(Xvalue+1)/2.0;
            if(gamepad1.b){
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);
                servoposition=strafe;
                frServo.setPosition(servoposition);
                flServo.setPosition(servoposition);
                brServo.setPosition(servoposition);
                blServo.setPosition(servoposition);
            }

            if(gamepad1.y){
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);
                servoposition=straight;
                frServo.setPosition(servoposition);
                flServo.setPosition(servoposition);
                brServo.setPosition(servoposition);
                blServo.setPosition(servoposition);
            }

            if(gamepad1.a){
                frServo.setPosition(0.05);
                flServo.setPosition(0.95);
                brServo.setPosition(0.95);
                blServo.setPosition(0.05);
                FR.setDirection(DcMotorSimple.Direction.FORWARD);
                BR.setDirection(DcMotorSimple.Direction.FORWARD);
            }

        }
    }

}
