package org.firstinspires.ftc.teamcode;

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
    private double HorizonSlide_minPosition = 0.0;
    private double HorizonSlide_position = 0.0;

    // * intake
    private Servo intakeLeft, intakeRight, intake_claw;
    private double intake_maxPosition = 0.9;
    private double intake_minPosition = 0.0;
    private double intake_pos = 0.0;
    private double intake_middlePosition = 0.3;
    private double intake_transferPosition = 0.87;

    double intake_ClawOpenPosition = 0.4;
    double intake_ClawClosePosition = 0.6;

    // * Slide
    DcMotorEx SlideLeft, SlideRight;
    private int SlideMaximumPosition = 3800;
    private int SlideMinimumPosition = 0;
    private int SlidePosition = 0;
    private int BasketSlidePosition = 3500;
    private double SlidePower = 0.6;

    // * Arm
    private Servo ArmLeft, ArmRight;
    private Servo OutputClaw;
    private static final double ARM_SERVO_MIN = 0.05;
    private static final double ARM_SERVO_MAX = 0.95;
    private static final double ARM_STAGE_DELAY_SECONDS = 0.35;
    private static final double ARM_START_ANGLE = 0.20;
    private static final double WRIST_START_OFFSET = 0.00;

    // Test these values in Test_Arm, then copy the safe armAngle/wristOffset here.
    private static final double ARM_TRANSFER_ANGLE = 0.255;
    private static final double WRIST_TRANSFER_OFFSET = 0.00;
    private static final double ARM_BASKET_ANGLE = 0.75;
    private static final double WRIST_BASKET_OFFSET = 0.23    ;
    private static final double ARM_SPECIMEN_ANGLE = 0.655;
    private static final double WRIST_SPECIMEN_OFFSET = 0.00;
    private static final double ARM_INTAKE_ANGLE = 0.23;
    private static final double WRIST_INTAKE_OFFSET = 0.00;

    private double armAngle = ARM_START_ANGLE;
    private double wristOffset = WRIST_START_OFFSET;
    private double armLeftTarget = ARM_START_ANGLE;
    private double armRightTarget = ARM_START_ANGLE;
    private int armStage = 1;
    private int lastMovementIndex = -1;
    private final ElapsedTime armStageTimer = new ElapsedTime();

    // * OutputClaw
    private double ClawOpenPosition = 0.3;
    private double ClawClosePosition = 0.65;

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
                HSR.setPosition(HorizonSlide_maxPosition);
                HSL.setPosition(HorizonSlide_maxPosition);
                intake_claw.setPosition(intake_ClawClosePosition);
            }

            // * -----------------------intake---------------------------- //
            // * intake position: 0.0 ~ 0.9

            if (HSR.getPosition() > 0.3 && HSL.getPosition() > 0.3) { // * 按下gamepad1.right_bumper，intake打開
                double intake_pos = intake_middlePosition - (gamepad1.right_trigger * intake_middlePosition);
                intake_pos = Math.min(intake_maxPosition, Math.max(intake_minPosition, intake_pos));
                intakeRight.setPosition(intake_pos);
                intakeLeft.setPosition(intake_pos);
            }

            // * -----------------------OutputClaw---------------------------- //
            // * OutputClaw Position: 0.3 ~ 0.65
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
                movementIndex = (movementIndex + 1) % 4; // 循環切換 0 到 3 的動作
                resetArmStage();
                buttonPressed = true; // 標記按鈕已被按下
            } else if (!gamepad2.left_bumper) {
                buttonPressed = false; // 當按鈕釋放時，重置狀態
            }

            if (movementIndex != lastMovementIndex) {
                resetArmStage();
                lastMovementIndex = movementIndex;
            }

            // 根據當前的動作索引執行對應的操作
            switch (movementIndex) {
                case 0:
                    OutputClaw.setPosition(ClawClosePosition);
                    break;
                case 1:
                    moveArmTwoStage(ARM_TRANSFER_ANGLE, WRIST_TRANSFER_OFFSET);
                    HSL.setPosition(0.03); // * Horizon Slide
                    HSR.setPosition(0.03);
                    intakeRight.setPosition(intake_transferPosition); // * intake
                    intakeLeft.setPosition(intake_transferPosition);
                    break;
                case 2:
                    SlideLeft.setTargetPosition(BasketSlidePosition);
                    SlideRight.setTargetPosition(BasketSlidePosition);
                    if(SlideLeft.getCurrentPosition() > 3000 && SlideRight.getCurrentPosition() > 3000){
                        moveArmTwoStage(ARM_BASKET_ANGLE, WRIST_BASKET_OFFSET);
                    }
                    break;
                case 3:
                    HSL.setPosition(0.25);
                    HSR.setPosition(0.25);
                    SlideLeft.setTargetPosition(1);
                    SlideRight.setTargetPosition(1);
                    moveArmTwoStage(ARM_INTAKE_ANGLE, WRIST_INTAKE_OFFSET);
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
            telemetry.addData("armAngle", "%.3f", armAngle);
            telemetry.addData("wristOffset", "%.3f", wristOffset);
            telemetry.addData("armStage", armStage);
            telemetry.addData("ArmLeft target", "%.3f", armLeftTarget);
            telemetry.addData("ArmRight target", "%.3f", armRightTarget);
            telemetry.addData("ArmLeftPosition", "%.3f", ArmLeft.getPosition());
            telemetry.addData("ArmRightPosition", "%.3f", ArmRight.getPosition());
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
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // * Horizon Slide
        HSL = hardwareMap.get(Servo.class, "servoLeft");
        HSR = hardwareMap.get(Servo.class, "servoRight");
        HSR.setDirection(Servo.Direction.REVERSE);

        HSL.setPosition(HorizonSlide_position);
        HSR.setPosition(HorizonSlide_position);

        // * intake
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intake_claw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeRight.setDirection(Servo.Direction.REVERSE);

        intakeRight.setPosition(intake_pos);
        intakeLeft.setPosition(intake_pos);

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
        setDifferentialArm(ARM_START_ANGLE, WRIST_START_OFFSET);

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

    private void moveArmTwoStage(double targetArmAngle, double targetWristOffset) {
        targetArmAngle = clamp(targetArmAngle, ARM_SERVO_MIN, ARM_SERVO_MAX);

        if (armStage == 0) {
            if (targetArmAngle > armAngle) {
                setDifferentialArm(targetArmAngle, wristOffset);
            } else if (targetArmAngle < armAngle) {
                setDifferentialArm(armAngle, targetWristOffset);
            } else {
                armStage = 1;
            }

            if (armStageTimer.seconds() >= ARM_STAGE_DELAY_SECONDS) {
                armStage = 1;
            }
        }

        if (armStage == 1) {
            setDifferentialArm(targetArmAngle, targetWristOffset);
        }
    }

    private void setDifferentialArm(double targetArmAngle, double targetWristOffset) {
        armAngle = clamp(targetArmAngle, ARM_SERVO_MIN, ARM_SERVO_MAX);
        wristOffset = targetWristOffset;
        armLeftTarget = clamp(armAngle + wristOffset, ARM_SERVO_MIN, ARM_SERVO_MAX);
        armRightTarget = clamp(armAngle - wristOffset, ARM_SERVO_MIN, ARM_SERVO_MAX);

        ArmLeft.setPosition(armLeftTarget);
        ArmRight.setPosition(armRightTarget);
    }

    private void resetArmStage() {
        armStage = 0;
        armStageTimer.reset();
    }

    private double clamp(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }
}
