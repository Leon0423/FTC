package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {

    //chassis
    private DcMotorEx FR = null, FL = null, BR = null, BL = null;
    private double fl, fr, bl, br;
    private double drive_speed = 0.7 ;

    //slide
    private DcMotorEx slide_left = null, slide_right = null;
    private int slidepos = 0;
    private double slide_speed = 0.6;
    private int maxEncoderValue = 2800;
    private int minEncoderValue = 0;
    private ElapsedTime xTimer = new ElapsedTime();
    private double xInterval = 1;

    //intake
    private DcMotorEx intake = null;
    private boolean intakeRunning = false; // 定義一個變數來追蹤 intake 的狀態

    //claw
    private Servo clawServo = null;
    // 伺服馬達的初始位置
    private double clawPosition = 0.5;
    // 用來切換夾子狀態的變數
    private boolean toggleButtonPressed = false;

    //drone
    private Servo drone = null;
    private double dronePosition = 0.65;

    //arm
    private Servo armServo = null;
    private double initialPosition = 0; // 初始位置
    private double forwardPosition = 0.3; // 正轉位置


    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        while(!isStarted()) {
            init_telemetry();
        }
        waitForStart();
        while(opModeIsActive()){
            init_telemetry();

            drive();

            slide();

            intake();

            claw();

            drone();

            arm();

            idle();
        }
    }

    public  void drive(){
        //drive_part
        double drive = -gamepad1.left_stick_y;     //前進
        double strafe = -gamepad1.left_stick_x;    //平移
        double turn = gamepad1.right_stick_x;      //自旋

        fr = (-drive - strafe + turn) * drive_speed;
        fl = (-drive + strafe - turn) * drive_speed;
        br = (-drive + strafe + turn) * drive_speed;
        bl = (-drive - strafe - turn) * drive_speed;

        double scale = scaling_power(fr, fl, br, bl);
        FR.setPower(fr / scale);
        FL.setPower(fl / scale);
        BR.setPower(br / scale);
        BL.setPower(bl / scale);
    }

    public void slide(){
        //Slide_part
        double rightTriggerValue = gamepad1.right_trigger;
        double leftTriggerValue = gamepad1.left_trigger;

        slide_left.setTargetPosition( slidepos );
        slide_right.setTargetPosition( slidepos );

        if (rightTriggerValue > 0 && slidepos < maxEncoderValue && slide_left.getCurrentPosition() < maxEncoderValue && slide_right.getCurrentPosition() < maxEncoderValue) {
            slidepos += 10;
            slide_right.setPower(slide_speed);
            slide_left.setPower(slide_speed);
        } else if (leftTriggerValue > 0 && slidepos > minEncoderValue && slide_left.getCurrentPosition() > minEncoderValue && slide_right.getCurrentPosition() > minEncoderValue) {
            slidepos -= 10;
            slide_right.setPower(slide_speed);
            slide_left.setPower(slide_speed);
        }

        if (gamepad1.y) {
            if (xTimer.seconds() >= xInterval) {
                slide_speed += 0.05;
                xTimer.reset();
            }
        } else if (gamepad1.x) {
            if (xTimer.seconds() >= xInterval) {
                slide_speed -= 0.05;
                xTimer.reset();
            }
        }

    }

    public void intake() {
        if (gamepad1.a && !intakeRunning) {
            intake.setPower(0.75); // 正轉
            intakeRunning = true;
        } else if (gamepad1.a && intakeRunning) {
            intake.setPower(0.0); // 停止
            intakeRunning = false;
        }
        // 等待按鍵釋放，避免連續多次啟動或停止
        while (gamepad1.a) {
            // 等待按鍵釋放
            idle();
        }

        if (gamepad1.b && !intakeRunning) {
            intake.setPower(-0.75); // 正轉
            intakeRunning = true;
        } else if (gamepad1.b && intakeRunning) {
            intake.setPower(0.0); // 停止
            intakeRunning = false;
        }
        while (gamepad1.b) {
            // 等待按鍵釋放
            idle();
        }
    }

    public void claw(){
        // 使用遙控器的按鈕控制伺服馬達的位置
        if (gamepad1.right_bumper && !toggleButtonPressed) {
            // 切換夾子狀態
            if (clawPosition == 0.5) {
                clawPosition = 0.65; // 設定為張開的位置
            } else {
                clawPosition = 0.5; // 設定為夾緊的位置
            }

            // 設定按鈕為已按下，避免連續變更
            toggleButtonPressed = true;
        } else if (!gamepad1.right_bumper) {
            // 如果按鈕釋放，設定按鈕為未按下
            toggleButtonPressed = false;
        }

        // 設定伺服馬達的位置
        clawServo.setPosition(clawPosition);

        // 讓控制迴圈平穩執行
        idle();
    }

    public void arm(){
        if (gamepad1.dpad_up) {
            armServo.setPosition(forwardPosition); // 正轉
        } else if (gamepad1.dpad_down) {
            armServo.setPosition(initialPosition); // 回到初始位置
        }
    }

    public void drone(){
        if(gamepad1.left_bumper){
            drone.setPosition(dronePosition);
        }
    }


    public void init_hardware(){
        //chassis_Base
        FR=hardwareMap.get(DcMotorEx.class, "FR");
        FL=hardwareMap.get(DcMotorEx.class, "FL");
        BR=hardwareMap.get(DcMotorEx.class, "BR");
        BL=hardwareMap.get(DcMotorEx.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(BRAKE);
        BL.setZeroPowerBehavior(BRAKE);
        FR.setZeroPowerBehavior(BRAKE);
        BR.setZeroPowerBehavior(BRAKE);

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
        armServo.setDirection(Servo.Direction.REVERSE);
        armServo.setPosition(initialPosition);

    }

    public double scaling_power(double fr, double fl, double br, double bl){
        double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
        if (max <= 1){
            max = 1;
        }
        return max;
    }

    public void init_telemetry(){
        //Slide
        telemetry.addData("slidepos", slidepos);
        telemetry.addData("Slide_right", slide_right.getCurrentPosition());
        telemetry.addData("Slide_left", slide_left.getCurrentPosition());
        telemetry.addData("left_trigger", gamepad1.left_trigger);
        telemetry.addData("right_trigger", gamepad1.right_trigger);
        telemetry.addData("Slide_speed", slide_speed);


        //drive
        telemetry.addData("drive_speed", drive_speed);
        telemetry.addData("fl", fl);
        telemetry.addData("bl", bl)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ;
        telemetry.addData("fr", fr);
        telemetry.addData("br", br);
        telemetry.addData("left_stick_x", -gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", -gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);

        //claw
        telemetry.addData("clawPosition",clawPosition);
        telemetry.addData("claw", clawServo.getPosition());

        telemetry.update();
    }
}