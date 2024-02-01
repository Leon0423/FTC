package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    //slide
    private DcMotorEx slide_left = null, slide_right = null;
    private int slidepos = 0;
    private double slide_speed = 0.6;
    private int maxEncoderValue = 2800;
    private int minEncoderValue = 0;

    //intake
    private DcMotorEx intake = null;

    //claw
    private Servo clawServo = null;
    // 伺服馬達的初始位置
    private double clawPosition = 0.65;

    //drone
    private Servo drone = null;

    //arm
    private Servo armServo = null;
    private double initialPosition = 0; // 初始位置
    public void init_hardware(){

        //slide_Base
        slide_left=hardwareMap.get(DcMotorEx.class, "SL");
        slide_right=hardwareMap.get(DcMotorEx.class, "SR");
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_right.setZeroPowerBehavior(BRAKE);
        slide_left.setZeroPowerBehavior(BRAKE);

        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_left.setTargetPosition(0);
        slide_right.setTargetPosition(0);

        slidepos = Math.min(Math.max(slidepos, minEncoderValue), maxEncoderValue);


        //intake_Base
        intake=hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(FLOAT);
        intake.setPower(0);

        //Servo_Base
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServo.setPosition(clawPosition);

        drone = hardwareMap.get(Servo.class, "drone");
        drone.setPosition(0.55);

        armServo = hardwareMap.get(Servo.class, "arm");
        armServo.setPosition(initialPosition);

    }

}
