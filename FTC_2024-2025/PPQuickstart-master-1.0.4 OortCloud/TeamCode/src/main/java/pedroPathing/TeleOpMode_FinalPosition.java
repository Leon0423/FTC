package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode_FinalPosition")
public class TeleOpMode_FinalPosition extends LinearOpMode {
    // * chassis
    private DcMotorEx FR, FL, BR, BL;
    private double drive, turn, strafe;
    private double fr, fl, br, bl, scale;

    // * Horizon Slide
    private Servo HSR, HSL;
    private double HorizonSlide_maxPosition = 0.43;
    private double HorizonSlide_minPosition = 0.01;

    // * intake
    private Servo intakeLeft, intakeRight, intake_claw;
    private double intake_maxPosition = 0.8;
    private double intake_minPosition = 0.0;
    private double intake_middlePosition = 0.3;
    private double intake_transferPosition = 0.87;

    double intake_ClawOpenPosition = 0.3;
    double intake_ClawClosePosition = 0.5;

    // * Slide
    DcMotorEx SlideLeft, SlideRight;
    private int SlideMaximumPosition = 3600;
    private int SlideMinimumPosition = 0;
    private int SlidePosition = 0;
    private int BasketSlidePosition = 3500;
    private double SlidePower = 0.8;

    // * Arm
    private Servo ArmLeft, ArmRight;
    private Servo OutputClaw;
    private double Arm_Position = 0;
    private double ArmInitialPosition = 0.0;
    private double ArmBasketPosition = 0.9;
    private double ArmTransferPosition = 0.15;

    // * OutputClaw
    private double ClawOpenPosition = 0.7;
    private double ClawClosePosition = 0.5;

    // * transferPosition
    private int movementIndex = 0; // 用於追蹤當前的動作索引
    private boolean buttonPressed = false; // 防止按鈕重複觸發

    @Override
    public void runOpMode() throws InterruptedException {
        init_hardware();
        waitForStart();
        while (opModeIsActive()) {
            // * ---------------------------Chassis---------------------------//
            drive = -gamepad1.left_stick_y;     // * 前進
            turn = gamepad1.right_stick_x;      // * 自旋
            strafe = gamepad1.left_stick_x;    // * 平移

            fr = drive - turn - strafe;
            fl = drive + turn + strafe;
            br = drive - turn + strafe;
            bl = drive + turn - strafe;

            scale = scaling_power(fr, fl, br, bl); // * 取得最大值

            FR.setPower((fr / scale) * 0.6);
            FL.setPower((fl / scale) * 0.6);
            BR.setPower((br / scale) * 0.6);
            BL.setPower((bl / scale) * 0.6);

            // * -------------------------Horizon Slide-------------------------- //
            // * Horizon Slide Position: 0.0 ~ 0.43
            if (gamepad1.right_bumper) { // * 按下gamepad1.right_bumper
                HSR.setPosition(0.43);
                HSL.setPosition(0.43);
                intake_claw.setPosition(intake_ClawClosePosition);
            }

            // * -----------------------intake---------------------------- //
            // * intake position: 0.0 ~ 0.8

            if (HSR.getPosition() > 0.3 && HSL.getPosition() > 0.3) { // * 按下gamepad1.right_bumper，intake打開
                double intake_pos = 0.3 - (gamepad1.right_trigger * 0.3);
                intakeRight.setPosition(intake_pos);
                intakeLeft.setPosition(intake_pos);
            }

            // * -----------------------OutputClaw---------------------------- //
            // * OutputClaw Position: 0.68 ~ 0.85
            if(gamepad1.dpad_up){
                intake_claw.setPosition(intake_ClawOpenPosition);
            }
            if (gamepad1.dpad_down){
                intake_claw.setPosition(intake_ClawClosePosition);
            }
            if(gamepad1.dpad_right){
                OutputClaw.setPosition(ClawOpenPosition);
            }
            if(gamepad1.dpad_left){
                OutputClaw.setPosition(ClawClosePosition);
            }

            // * -----------------------TransferPosition---------------------------- //

            // 檢查按鈕是否被按下

            if (gamepad2.left_bumper && !buttonPressed) {
                movementIndex = (movementIndex + 1) % 4; // 循環切換 0 到 4 的動作
                buttonPressed = true; // 標記按鈕已被按下
            } else if (!gamepad2.left_bumper) {
                buttonPressed = false; // 當按鈕釋放時，重置狀態
            }

            // 根據當前的動作索引執行對應的操作
            switch (movementIndex) {
                case 0:
                    OutputClaw.setPosition(ClawClosePosition);
                    break;
                case 1:
                    ArmRight.setPosition(0.23); // * Arm
                    ArmLeft.setPosition(0.23);
                    HSL.setPosition(0.03); // * Horizon Slide
                    HSR.setPosition(0.03);
                    intakeRight.setPosition(0.8); // * intake
                    intakeLeft.setPosition(0.8);
                    break;
                case 2:
                    SlideLeft.setTargetPosition(3600);
                    SlideRight.setTargetPosition(3600);
                    if(SlideLeft.getCurrentPosition() > 3000 && SlideRight.getCurrentPosition() > 3000){
                        ArmLeft.setPosition(0.75);
                        ArmRight.setPosition(0.75);
                    }
                    break;
                case 3:
                    HSL.setPosition(0.25);
                    HSL.setPosition(0.25);
                    SlideLeft.setTargetPosition(1);
                    SlideRight.setTargetPosition(1);
                    ArmRight.setPosition(0.23); // * Arm
                    ArmLeft.setPosition(0.23);
                    intake_claw.setPosition(intake_ClawClosePosition);
                    break;
            }

            // * -----------------------Telemetry---------------------------- //

            // * Horizon Slide
            telemetry.addData("HSR", HSR.getPosition());
            telemetry.addData("HSL", HSL.getPosition());
            telemetry.addData("HorizonSlide_position", HSR.getPosition());

            // * intake
            telemetry.addData("intakeRight", intakeRight.getPosition());
            telemetry.addData("intakeLeft", intakeLeft.getPosition());
            telemetry.addLine(" ");

            telemetry.addData("intake_claw", intake_claw.getPosition());
            telemetry.addLine(" ");

            // * Slide
            telemetry.addData("Left Encoder Position", SlideLeft.getCurrentPosition());
            telemetry.addData("Right Encoder Position", SlideRight.getCurrentPosition());
            telemetry.addLine(" ");

            // * Arm
            telemetry.addData("ArmLeftPosition", ArmLeft.getPosition());
            telemetry.addData("ArmRightPosition", ArmRight.getPosition());
            telemetry.addLine(" ");

            //* OutputClaw
            telemetry.addLine("OutputClaw: dpad_up / dpad_down");
            telemetry.addData("OutputClawPosition", OutputClaw.getPosition());
            telemetry.addLine(" ");

            telemetry.update();
        }
    }
        public void init_hardware () {
            // * chassis
            FR = hardwareMap.get(DcMotorEx.class, "FR");
            FL = hardwareMap.get(DcMotorEx.class, "FL");
            BR = hardwareMap.get(DcMotorEx.class, "BR");
            BL = hardwareMap.get(DcMotorEx.class, "BL");

            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);

            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // * Horizon Slide
            HSL = hardwareMap.get(Servo.class, "servoLeft");
            HSR = hardwareMap.get(Servo.class, "servoRight");
            HSR.setDirection(Servo.Direction.REVERSE);

            HSL.setPosition(0.08);
            HSR.setPosition(0.08);

            // * intake
            intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
            intakeRight = hardwareMap.get(Servo.class, "intakeRight");
            intake_claw = hardwareMap.get(Servo.class, "intakeClaw");
            intakeRight.setDirection(Servo.Direction.REVERSE);

            intakeRight.setPosition(0.6);
            intakeLeft.setPosition(0.6);

            intake_claw.setPosition(intake_ClawClosePosition);

            // * Slide
            SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
            SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
            SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            SlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

            SlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            SlideLeft.setTargetPosition(SlidePosition);
            SlideRight.setTargetPosition(SlidePosition);
            SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideLeft.setTargetPosition(SlidePosition);
            SlideRight.setTargetPosition(SlidePosition);
            SlideLeft.setPower(SlidePower);
            SlideRight.setPower(SlidePower);

            // * Arm
            ArmLeft = hardwareMap.get(Servo.class, "ArmLeft");
            ArmRight = hardwareMap.get(Servo.class, "ArmRight");
            ArmRight.setDirection(Servo.Direction.REVERSE);
            ArmLeft.setPosition(ArmInitialPosition);
            ArmRight.setPosition(ArmInitialPosition);

            // * OutputClaw
            OutputClaw = hardwareMap.get(Servo.class, "OutputClaw");
            OutputClaw.setPosition(ClawClosePosition);

        }
        public double scaling_power ( double fr, double fl, double br, double bl){
            double max = Math.max(Math.max(Math.abs(fr), Math.abs(fl)), Math.max(Math.abs(br), Math.abs(bl)));
            if (max <= 1) {
                max = 1;
            }
            return max;
        }
    }