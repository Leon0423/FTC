package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="AbsoluteDirectionDrive_new")
public class AbsoluteDirection_new extends LinearOpMode {

    //set motor & servo & imu
    private DcMotorEx FL, BL, FR, BR;
    private BNO055IMU imu;

    double imu_x, imu_y, imu_z;

    //set variable
    double drive_o, drive_y, drive_x;
    private double drive_speed = 0.7;
    private double angle;
    double yFL, yBL, yFR, yBR;
    double xFL, xBL, xFR, xBR;

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        while (!isStarted()) {
            init_telemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            init_telemetry();
            AbsoluteControls();
        }
    }

    public void init_hardware(){
        //get motor & servo
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        //motor encoder set
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set motor reverse
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        //set motor stop action
        FL.setZeroPowerBehavior(BRAKE);
        BL.setZeroPowerBehavior(BRAKE);
        FR.setZeroPowerBehavior(BRAKE);
        BR.setZeroPowerBehavior(BRAKE);

        init_imu();
    }

    public void init_imu() {
        //set imu
        BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();

        //TODO: check if it help or not
        // IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // IMU_Parameters.calibrationDataFile = "BNO055IMUCalibration. json";

        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU_Parameters.calibrationDataFile = "BNO055IMUCalibration. json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize (IMU_Parameters);
    }

    public void AbsoluteControls(){
        //get gamepad information
        drive_o = gamepad1.right_stick_x;
        drive_y = gamepad1.left_stick_y;
        drive_x = gamepad1.left_stick_x;

        imu_x = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        imu_y = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).secondAngle;
        imu_z = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).thirdAngle;

        //get imu information
        //TODO: check AxesOrder.ZXY or something
        angle = -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);

        //imu angle calculate
        if ( angle > 180 ){
            angle = angle - 360;
        }
        if ( angle < -180 ){
            angle = angle + 360;
        }
        if ( angle >= -45 && angle <= 135 ){
            yFL = -( angle / 45 - 1 );
            yBR = -( angle / 45 - 1 );
            xBL =  ( angle / 45 ) - 1;
            xFR =  ( angle / 45 ) - 1;
        } else if ( angle > 0 ){
            angle = angle - 360;
            yFL =  ( angle / 45 ) + 3;
            yBR =  ( angle / 45 ) + 3;
            xBL = -( angle / 45+ 3 );
            xFR = -( angle / 45+ 3 );
        } else {
            yFL =  ( angle / 45 ) + 3;
            yBR =  ( angle / 45 ) + 3;
            xBL = -( angle / 45 + 3 );
            xFR = -( angle / 45 + 3 );
        }
        if ( angle >= -135 && angle <= 45 ){
            yBL =  ( angle / 45 ) + 1;
            yFR =  ( angle / 45 ) + 1;
            xFL =  ( angle / 45 ) + 1;
            xBR =  ( angle / 45 ) + 1;
        } else if ( angle < 0 ){
            angle = angle + 360;
            yBL = -( angle / 45 - 3 );
            yFR = -( angle / 45 - 3 );
            xFL = -( angle / 45 - 3 );
            xBR = -( angle / 45 - 3 );
        } else {
            yBL = -( angle / 45 - 3 );
            yFR = -( angle / 45 - 3 );
            xFL = -( angle / 45 - 3 );
            xBR = -( angle / 45 - 3 );
        }

        //wheels caculate
        FR.setPower(( drive_y * yFR - drive_x * xFR + drive_o) * drive_speed );
        BR.setPower(( drive_y * yBR - drive_x * xBR + drive_o) * drive_speed );
        FL.setPower(( drive_y * yFL - drive_x * xFL - drive_o) * drive_speed );
        BL.setPower(( drive_y * yBL - drive_x * xBL - drive_o) * drive_speed );
    }

    public void init_telemetry(){
        telemetry.addData("angle", angle);
        telemetry.addData("Pitch_x", "%.2f", imu_x);
        telemetry.addData("Roll_y", "%.2f", imu_y);
        telemetry.addData("Yaw_z", "%.2f", imu_z);
        telemetry.addData("FL", FL.getCurrentPosition());
        telemetry.addData("BL", BL.getCurrentPosition());
        telemetry.addData("FR", FR.getCurrentPosition());
        telemetry.addData("BR", BR.getCurrentPosition());
        telemetry.update();
    }
}