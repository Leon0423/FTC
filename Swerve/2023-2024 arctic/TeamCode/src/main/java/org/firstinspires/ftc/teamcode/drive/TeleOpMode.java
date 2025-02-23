package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends TeleOp_Base{
    double drive, turn, strafe;
    double fr, fl, br, bl, scale;
    int slideR_pos=0,slideL_pos=0,clamp_pos=0;
    double power=0.8;//變數設定r

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        claw.setPosition(0.97);
        clampingservo.setPosition(0.05);
        clawR.setPosition(0);
        clawL.setPosition(0);
        shooter.setPosition(0.5);

        waitForStart();
        //初始狀態設定，例如Servo初始位置
        while(opModeIsActive()) {
            telemetry.addData("slideR_pos", slideR_pos);
            telemetry.addData("slideL_pos", slideL_pos);
            telemetry.addData("clamp_pos", clamp_pos);
            telemetry.addData( "fr",FR.getCurrentPosition());
            telemetry.addData( "fl",FL.getCurrentPosition());
            telemetry.addData( "br",BR.getCurrentPosition());
            telemetry.addData( "bl",BL.getCurrentPosition());
            telemetry.update();

            drive = -gamepad1.left_stick_y;     //前進
            turn = gamepad1.right_stick_x;      //自旋
            strafe = gamepad1.left_stick_x;    //平移
            fr = drive-turn-strafe;
            fl = drive+turn+strafe;
            br = drive-turn+strafe;
            bl = drive+turn-strafe;
            scale = scaling_power(fr, fl, br, bl);
            FR.setPower(fr/scale*power);
            FL.setPower(fl/scale*power);
            BR.setPower(br/scale*power);
            BL.setPower(bl/scale*power);


            //shooter
            if(gamepad1.dpad_left){
                shooter.setPosition(0);
            }

            //claw
            if(gamepad1.x){
                claw.setPosition(0.85);
            }
            else if(gamepad1.b){
                claw.setPosition(1);
            }

            //servoR.L
            if(slideR_pos>1000 && slideL_pos>1000){
                clawR.setPosition(0.28);
                clawL.setPosition(0.28);
            }
            else if(slideR_pos<1000 && slideL_pos<1000){
                clawR.setPosition(0);
                clawL.setPosition(0);
            }

            //clamp&clampingservo
            if(gamepad1.dpad_up){
                clampingservo.setPosition(0.45);
                clamp_pos=12500;
            }
            else if(gamepad1.dpad_down){
                clamp_pos=0;
            }
            clamp.setTargetPosition(clamp_pos);
            if(clamp_pos==12500){
                clampingservo.setPosition(0.5);
            }

            //slide
            if(gamepad1.right_bumper && slideR_pos<2300 && slideL_pos<2300){
                slideR_pos+=30;
                slideL_pos+=30;
            }
            else if(gamepad1.left_bumper && slideR_pos>0 && slideL_pos>0){
                slideR_pos-=30;
                slideL_pos-=30;
            }
            SR.setTargetPosition(slideR_pos);
            SL.setTargetPosition(slideL_pos);

            //intake
            if(gamepad1.y){
                intake.setPower(1);
            }
            else if (gamepad1.a){
                intake.setPower(-1);
            }

            else
                intake.setPower(0);




        }


    }
}
