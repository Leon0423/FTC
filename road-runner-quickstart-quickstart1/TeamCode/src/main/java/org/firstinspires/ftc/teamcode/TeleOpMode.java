package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {

    private DcMotorEx FR = null, FL = null, BR = null, BL = null;
    private double fl, fr, bl, br;

    //slide
    private DcMotorEx Slide_left = null, Slide_right = null;
    private DcMotorEx intake = null;
    private double drive_speed = 1 ;
    private double Slide_speed = 0.1;
    private int MAX_Slide_pos = 50;
    private int incrementAngle;
    private int currentRotation;
    private int targetRotation;
    private boolean intakeRunning = false; // 定義一個變數來追蹤 intake 的狀態

    //servo
    private Servo clawServo = null;
    // 伺服馬達的初始位置
    private double clawPosition = 0.5;
    // 用來切換夾子狀態的變數
    private boolean toggleButtonPressed = false;

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

        incrementAngle = 5; // 每次上升的固定角度
        currentRotation = Slide_left.getCurrentPosition(); // 當前的馬達位置
        targetRotation = currentRotation + (int) (incrementAngle * (rightTriggerValue - leftTriggerValue));

        // 限制目標旋轉度數在0到MAX_Slide_pos之間
        targetRotation = Math.min(Math.max(targetRotation, 0), MAX_Slide_pos);

        // 設定馬達的目標Encoder位置
        Slide_left.setTargetPosition(targetRotation);
        Slide_right.setTargetPosition(targetRotation);

        // 啟動馬達運動
        Slide_left.setPower((rightTriggerValue - leftTriggerValue) * Slide_speed);  // 設定左馬達功率
        Slide_right.setPower((rightTriggerValue - leftTriggerValue) * Slide_speed); // 設定右馬達功率

        // 等待馬達到達目標位置
        while (Slide_left.isBusy() || Slide_right.isBusy()) {
            // 在這裡等待，不執行其他動作
        }

        // 運動完成後停止馬達
        if ()
        Slide_left.setPower(0);
        Slide_right.setPower(0);
    }

    public void intake() {
        // 在遙控器的按鍵 A 上設定的按鍵，這裡以 gamepad1 的 A 按鍵為例
        if (gamepad1.a) {
            // 如果 A 按鍵被按下

            if (!intakeRunning) {
                // 如果 intake 沒有在運行，啟動 intake
                intake.setPower(1.0);  // 可以根據需要調整功率
                intakeRunning = true;
            } else {
                // 如果 intake 已經在運行，停止 intake
                intake.setPower(0);
                intakeRunning = false;
            }

            // 等待按鍵釋放，避免連續多次啟動或停止
            while (gamepad1.a) {
                // 等待按鍵釋放
                idle();
            }
        }
    }

    public void claw(){
        // 使用遙控器的按鈕控制伺服馬達的位置
        if (gamepad1.a && !toggleButtonPressed) {
            // 切換夾子狀態
            if (clawPosition == 0.0) {
                clawPosition = 1.0; // 設定為張開的位置
            } else {
                clawPosition = 0.0; // 設定為夾緊的位置
            }

            // 設定按鈕為已按下，避免連續變更
            toggleButtonPressed = true;
        } else if (!gamepad1.a) {
            // 如果按鈕釋放，設定按鈕為未按下
            toggleButtonPressed = false;
        }

        // 設定伺服馬達的位置
        clawServo.setPosition(clawPosition);

        // 讓控制迴圈平穩執行
        idle();
    }

    public void init_hardware(){
        //chassis_Base
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

        //slide_Base
        Slide_left=hardwareMap.get(DcMotorEx.class, "SL");
        Slide_right=hardwareMap.get(DcMotorEx.class, "SR");
        Slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide_left.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide_right.setZeroPowerBehavior(BRAKE);
        Slide_left.setZeroPowerBehavior(BRAKE);

        Slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slide_right.setTargetPosition(0);
        Slide_left.setTargetPosition(0);

        //intake_Base
        intake=hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(FLOAT);
        intake.setPower(0);

        // Servo_Base
        clawServo = hardwareMap.get(Servo.class, "claw");
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
        telemetry.addData("Slide_right", Slide_right.getCurrentPosition());
        telemetry.addData("Slide_left", Slide_left.getCurrentPosition());
        telemetry.addData("left_trigger", gamepad1.left_trigger);
        telemetry.addData("right_trigger", gamepad1.right_trigger);
        telemetry.addData("incrementAngle", incrementAngle);
        telemetry.addData("currentRotation", currentRotation);
        telemetry.addData("targetRotation", targetRotation);
        telemetry.addData("Slide_speed", Slide_speed);

        //drive
        telemetry.addData("drive_speed", drive_speed);
        telemetry.addData("fl", fl);
        telemetry.addData("bl", bl)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ;
        telemetry.addData("fr", fr);
        telemetry.addData("br", br);
        telemetry.addData("left_stick_x", -gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", -gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.update();

        //claw
    }
}