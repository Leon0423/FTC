package org.firstinspires.ftc.teamcode.drive;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class TeleOp_Base extends LinearOpMode {
    public DcMotorEx FR, FL, BR, BL, SR,SL,intake,clamp;
    public Servo claw,clawR,clawL,clampingservo,shooter; //創建物件
    public void init_hardware() {


        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");
        SL = hardwareMap.get(DcMotorEx.class, "SL");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        clamp = hardwareMap.get(DcMotorEx.class, "clamp");
        claw = hardwareMap.get(Servo.class, "claw");
        clawR = hardwareMap.get(Servo.class, "clawR");
        clawL = hardwareMap.get(Servo.class, "clawL");
        shooter = hardwareMap.get(Servo.class, "shooter");
        clampingservo = hardwareMap.get(Servo.class, "clampingservo");
        clawL.setDirection(Servo.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clamp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clamp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clamp.setTargetPosition(0);
        clamp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clamp.setTargetPosition(0);
        clamp.setPower(1);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setTargetPosition(0);
        SR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SR.setTargetPosition(0);
        SR.setPower(1);
        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SL.setTargetPosition(0);
        SL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SL.setTargetPosition(0);
        SL.setPower(1);
        SL.setDirection(DcMotorSimple.Direction.REVERSE);//設定物件
    }
    public double scaling_power(double fr, double fl, double br, double bl) {
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if (max<=1) {
            max=1;
        }
        return max;
    }
}