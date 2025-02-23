package org.firstinspires.ftc.teamcode.drive.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SwerveDrive_Base extends LinearOpMode {
    public DcMotorEx FR, FL, BR, BL;
    public Servo frServo,flServo,brServo,blServo;
    public void init_hardware(){

        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        frServo = hardwareMap.get(Servo.class, "frServo");
        flServo = hardwareMap.get(Servo.class, "flServo");
        brServo = hardwareMap.get(Servo.class, "brServo");
        blServo = hardwareMap.get(Servo.class, "blServo");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
