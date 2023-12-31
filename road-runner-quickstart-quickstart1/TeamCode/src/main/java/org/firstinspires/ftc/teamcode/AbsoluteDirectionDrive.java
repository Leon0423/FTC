package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "AbsoluteDirectionDrive")
public class AbsoluteDirectionDrive extends AbsoluteDirectionDrive_Base {

    //set variable
    double drive_speed;
    double drive_speed_max = 0.7;
    double angle;
    double yFL;
    double yBL;
    double yFR;
    double yBR;
    double xFL;
    double xBL;
    double xFR;
    double xBR;


    @Override
    public void runOpMode() throws InterruptedException {

        init_hardware();

        waitForStart();

        while(opModeIsActive()) {

            //get Gamepad information
            double drive_o = gamepad1.right_stick_x;
            double drive_y = gamepad1.left_stick_y;
            double drive_x = gamepad1.left_stick_x;

            //get imu information
            angle = -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);

            //gamepad caculate
            double boxy = gamepad1.left_stick_y;
            double boxx = gamepad1.left_stick_x;
            double rightstickx = Math.abs( gamepad1.right_stick_x );
            double leftstick = Math.abs( Math.sqrt( Math.pow( boxy, 2 ) + Math.pow( boxx, 2 ) ) );

            if ( leftstick < 0.2 ){
                drive_speed = 0;
            } else if ( leftstick < 0.5 ){
                drive_speed = 0.4;
            } else if ( leftstick < 1 ){
                drive_speed = drive_speed_max;
            }

            if ( rightstickx < 0.2 ){
                drive_speed = 0;
            } else if ( rightstickx < 0.5 ) {
                drive_speed = 0.4;
            } else if ( rightstickx < 1 ){
                drive_speed = drive_speed_max;
            }

            //imu angle calculate
            if (angle > 180){
                angle = angle - 360;
            }
            if (angle < -180){
                angle = angle + 360;
            }
            if (angle >= -45 && angle <= 135){
                yFL = -(angle/45-1);
                yBR = -(angle/45-1);
                xBL = (angle/45)-1;
                xFR = (angle/45)-1;
            } else if (angle > 0){
                angle = angle - 360;
                yFL = (angle/45)+3;
                yBR = (angle/45)+3;
                xBL = -(angle/45+3);
                xFR = -(angle/45+3);
            } else {
                yFL = (angle/45)+3;
                yBR = (angle/45)+3;
                xBL = -(angle/45+3);
                xFR = -(angle/45+3);
            }
            if (angle >= -135 && angle <= 45){
                yBL = (angle/45)+1;
                yFR = (angle/45)+1;
                xFL = (angle/45)+1;
                xBR = (angle/45)+1;
            } else if (angle < 0){
                angle = angle + 360;
                yBL = -(angle/45-3);
                yFR = -(angle/45-3);
                xFL = -(angle/45-3);
                xBR = -(angle/45-3);
            } else {
                yBL = -(angle/45-3);
                yFR = -(angle/45-3);
                xFL = -(angle/45-3);
                xBR = -(angle/45-3);
            }

            //wheels caculate
            FR.setPower( ( drive_y * yFR - drive_x * xFR + drive_o ) * drive_speed );
            BR.setPower( ( drive_y * yBR - drive_x * xBR + drive_o ) * drive_speed );
            FL.setPower( ( drive_y * yFL - drive_x * xFL - drive_o ) * drive_speed );
            BL.setPower( ( drive_y * yBL - drive_x * xBL - drive_o ) * drive_speed );

            //telemetry
            telemetry.addData("angle", angle);
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.addData("BL", BL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("BR", BR.getCurrentPosition());
            telemetry.update();
        }
    }
}
