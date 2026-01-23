package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class TeleOpMode_Base extends LinearOpMode {
    DcMotorEx FR, FL, BR, BL;

    //創建物件
    public void init_hardware() {
        //設定物件
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");


        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        FR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        //設定方向
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);

        idle();
    }
}
