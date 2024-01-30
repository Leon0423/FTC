package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class Autonomous_Base extends LinearOpMode {
    private DcMotorEx FR, FL, BR, BL;
    private BNO055IMU imu;
    public void init_hardware(){
        FR=hardwareMap.get(DcMotorEx.class, "FR");
        FL=hardwareMap.get(DcMotorEx.class, "FL");
        BR=hardwareMap.get(DcMotorEx.class, "BR");
        BL=hardwareMap.get(DcMotorEx.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setZeroPowerBehavior(BRAKE);
        BL.setZeroPowerBehavior(BRAKE);
        FR.setZeroPowerBehavior(BRAKE);
        BR.setZeroPowerBehavior(BRAKE);

        //set imu
        BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU_Parameters.calibrationDataFile = "BNO055IMUCalibration. json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize (IMU_Parameters);
    }

    public void DriveStraight(double distance, double heading, double speed, double kp){
        double error;
        double angle = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);
        double output;
        double LeftMotor = (FL.getCurrentPosition() + BL.getCurrentPosition()) / 2;
        double RightMotor = (FR.getCurrentPosition() + BR.getCurrentPosition()) / 2;

        //Fine tune Kp based on your robot design and speed
        FR.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);

        if (speed > 0){
            //Going forward
            while (LeftMotor < distance || RightMotor < distance){
                error = heading - angle;
                output = error * kp;
                FL.setPower((speed - output) * 0.01);
                BL.setPower((speed - output) * 0.01);
                FR.setPower((speed + output) * 0.01);
                BR.setPower((speed + output) * 0.01);
            }
        } else {
            //Going backward
            while (LeftMotor > distance || RightMotor > distance){
                error = heading - angle;
                output = error * kp;
                FL.setPower((speed - output) * 0.01);
                BL.setPower((speed - output) * 0.01);
                FR.setPower((speed + output) * 0.01);
                BR.setPower((speed + output) * 0.01);
            }
        }
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

}
