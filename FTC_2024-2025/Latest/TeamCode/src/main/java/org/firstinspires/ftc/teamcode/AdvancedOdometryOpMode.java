package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.AutoPathingGenerator.PP.pedroPathing.localization.Encoder;

/**
 * Advanced Odometry OpMode with Precise Offset Calculations
 * Uses offset measurements for more accurate position tracking
 */
@TeleOp(name="Odometry_Value", group="TEST_ODOMETRY")
public class AdvancedOdometryOpMode extends LinearOpMode {
    // Drive Motors
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // Odometry Encoders
    private DcMotor centerEncoder, leftEncoder, rightEncoder;

    // Odometry Constants
    private static final double TICKS_PER_REV = 8192; // Through Bore Encoder ticks
    private static final double WHEEL_DIAMETER = 1.26; // inches
    private static final double TICKS_TO_INCHES = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REV;

    // Critical Offset Measurements
    // IMPORTANT: Measure these precisely on your specific robot
    private static final double PARALLEL_Y = 7.0; // Distance between parallel wheels (inches)
    private static final double PERPENDICULAR_X = 5.8; // Lateral wheel offset from robot center (inches)

    // Robot Position Tracking
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;

    // Previous Encoder Positions
    private int prevCenterEncoderPos = 0;
    private int prevLeftEncoderPos = 0;
    private int prevRightEncoderPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Hardware
        initHardware();

        telemetry.addLine("Calibrate Starting Position");
        telemetry.addLine("Measure offsets carefully!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Mecanum Drive Control
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Mecanum Drive Kinematics
            leftFront.setPower(forward + strafe + turn);
            rightFront.setPower(forward - strafe - turn);
            leftRear.setPower(forward - strafe + turn);
            rightRear.setPower(forward + strafe - turn);

            // Reset Robot Position
            if (gamepad1.a) {
                resetPosition(0, 0, 0);
            }

            // Update Robot Position
            updateOdometry();

            // Telemetry with Precise Tracking
            telemetry.addData("Robot X", "%.2f inches", robotX);
            telemetry.addData("Robot Y", "%.2f inches", robotY);
            telemetry.addData("Robot Heading", "%.2f degrees", Math.toDegrees(robotHeading));

            // ToDo: add * -1 if needed to reverse the encoder values
            telemetry.addData("centerEncoder", centerEncoder.getCurrentPosition());
            telemetry.addData("leftEncoder", leftEncoder.getCurrentPosition());
            telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());

            telemetry.update();

            idle();
        }
    }

    private void updateOdometry() {
        // Get Current Encoder Positions
        int current_centerEncoder = centerEncoder.getCurrentPosition();
        int current_leftEncoder = leftEncoder.getCurrentPosition();
        int current_rightEncoder = rightEncoder.getCurrentPosition();

        // Calculate Encoder Deltas
        int deltaX = current_centerEncoder - prevCenterEncoderPos;
        int deltaY = (current_leftEncoder + current_rightEncoder) / 2 -
                (prevLeftEncoderPos + prevRightEncoderPos) / 2;

        // Convert Deltas to Inches
        double deltaXInches = deltaX * TICKS_TO_INCHES;
        double deltaYInches = deltaY * TICKS_TO_INCHES;

        // Calculate Heading Change
        // Note: This is a simplified heading calculation
        // For more precise heading, you might need a gyro
        double deltaHeading = Math.atan2(
                deltaXInches / PERPENDICULAR_X,
                deltaYInches / PARALLEL_Y
        );

        // Update Robot Position with Offset Compensation
        // This is a more mathematically rigorous approach
        robotX += deltaXInches * Math.cos(robotHeading) -
                deltaYInches * Math.sin(robotHeading);

        robotY += deltaXInches * Math.sin(robotHeading) +
                deltaYInches * Math.cos(robotHeading);

        // Update Heading
        robotHeading += deltaHeading;

        // Update Previous Encoder Positions
        prevCenterEncoderPos = current_centerEncoder;
        prevLeftEncoderPos = current_leftEncoder;
        prevRightEncoderPos = current_rightEncoder;
    }

    private void initHardware() {
        // Motor and Encoder Initialization (same as previous example)
        leftFront = hardwareMap.dcMotor.get("FL");
        rightFront = hardwareMap.dcMotor.get("FR");
        leftRear = hardwareMap.dcMotor.get("BL");
        rightRear = hardwareMap.dcMotor.get("BR");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset and Configure Encoders
        centerEncoder = hardwareMap.dcMotor.get("FL");
        leftEncoder = hardwareMap.dcMotor.get("BL");
        rightEncoder = hardwareMap.dcMotor.get("FR");

        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset Robot Position
        resetPosition(0, 0, 0);

        telemetry.addLine("Hardware Initialized");
        telemetry.update();
    }

    /**
     * Manually reset robot position
     * Useful for field positioning or autonomous alignment
     */
    private void resetPosition(double x, double y, double heading) {
        robotX = x;
        robotY = y;
        robotHeading = heading;

        // Reset encoders
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}