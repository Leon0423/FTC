package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MechanismFineTuner", group = "Test")
public class MechanismFineTuner extends LinearOpMode {
    private static final double SERVO_STEP = 0.005;
    private static final double SERVO_FINE_STEP = 0.001;
    private static final double SERVO_FAST_STEP = 0.020;

    private static final int SLIDE_STEP = 25;
    private static final int SLIDE_FINE_STEP = 5;
    private static final int SLIDE_FAST_STEP = 100;
    private static final int SLIDE_MIN_LIMIT = 0;
    private static final int SLIDE_MAX_LIMIT = 4000;
    private static final double SLIDE_POWER = 0.6;

    private Servo horizonLeft, horizonRight;
    private Servo intakeLeft, intakeRight, intakeClaw;
    private Servo armLeft, armRight, outputClaw;
    private DcMotorEx slideLeft, slideRight;

    private ServoChannel[] servoChannels;
    private int selectedServo = 0;

    private int slideTarget = 0;
    private int slideMinSeen = 0;
    private int slideMaxSeen = 0;
    private boolean slideMode = false;
    private boolean slowMode = true;

    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousStart = false;
    private boolean previousBack = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        showInitTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            handleModeButtons();

            if (slideMode) {
                tuneSlide();
            } else {
                tuneServo();
            }

            updateSeenLimits();
            showTelemetry();
            rememberButtons();
            idle();
        }
    }

    private void initHardware() {
        horizonLeft = hardwareMap.get(Servo.class, "servoLeft");
        horizonRight = hardwareMap.get(Servo.class, "servoRight");
        horizonRight.setDirection(Servo.Direction.REVERSE);

        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        intakeRight.setDirection(Servo.Direction.REVERSE);
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        armLeft = hardwareMap.get(Servo.class, "ArmLeft");
        armRight = hardwareMap.get(Servo.class, "ArmRight");
        armRight.setDirection(Servo.Direction.REVERSE);
        outputClaw = hardwareMap.get(Servo.class, "OutputClaw");

        slideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setTargetPosition(slideTarget);
        slideRight.setTargetPosition(slideTarget);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(SLIDE_POWER);
        slideRight.setPower(SLIDE_POWER);

        servoChannels = new ServoChannel[] {
                new ServoChannel("HorizonSlide", 0.08, 0.00, 0.43, horizonLeft, horizonRight),
                new ServoChannel("IntakeArm", 0.60, 0.00, 0.87, intakeLeft, intakeRight),
                new ServoChannel("IntakeClaw", 0.50, 0.30, 0.60, intakeClaw),
                new ServoChannel("OutputArm", 0.20, 0.00, 0.90, armLeft, armRight),
                new ServoChannel("OutputClaw", 0.50, 0.30, 0.70, outputClaw)
        };

        for (ServoChannel channel : servoChannels) {
            channel.apply();
        }
    }

    private void handleModeButtons() {
        if (pressedY()) {
            slideMode = !slideMode;
        }

        if (pressedX()) {
            slowMode = !slowMode;
        }

        if (!slideMode) {
            if (pressedDpadRight()) {
                selectedServo = (selectedServo + 1) % servoChannels.length;
            }
            if (pressedDpadLeft()) {
                selectedServo = (selectedServo - 1 + servoChannels.length) % servoChannels.length;
            }
        }
    }

    private void tuneServo() {
        ServoChannel channel = servoChannels[selectedServo];
        double step = currentServoStep();
        double delta = 0.0;

        if (gamepad2.dpad_up) {
            delta += step;
        }
        if (gamepad2.dpad_down) {
            delta -= step;
        }
        if (gamepad2.right_trigger > 0.2) {
            delta += SERVO_FAST_STEP * gamepad2.right_trigger;
        }
        if (gamepad2.left_trigger > 0.2) {
            delta -= SERVO_FAST_STEP * gamepad2.left_trigger;
        }

        if (delta != 0.0) {
            channel.move(delta);
        }

        if (pressedA()) {
            channel.set(channel.recommendedMin);
        }
        if (pressedB()) {
            channel.set(channel.recommendedMax);
        }
        if (pressedStart()) {
            channel.resetSeenLimits();
        }
    }

    private void tuneSlide() {
        int step = currentSlideStep();

        if (gamepad2.dpad_up) {
            slideTarget += step;
        }
        if (gamepad2.dpad_down) {
            slideTarget -= step;
        }
        if (gamepad2.right_trigger > 0.2) {
            slideTarget += (int) (SLIDE_FAST_STEP * gamepad2.right_trigger);
        }
        if (gamepad2.left_trigger > 0.2) {
            slideTarget -= (int) (SLIDE_FAST_STEP * gamepad2.left_trigger);
        }
        if (pressedA()) {
            slideTarget = SLIDE_MIN_LIMIT;
        }
        if (pressedB()) {
            slideTarget = SLIDE_MAX_LIMIT;
        }
        if (pressedStart()) {
            slideMinSeen = averageSlidePosition();
            slideMaxSeen = slideMinSeen;
        }
        if (pressedBack()) {
            resetSlideEncoder();
        }

        slideTarget = clip(slideTarget, SLIDE_MIN_LIMIT, SLIDE_MAX_LIMIT);
        slideLeft.setTargetPosition(slideTarget);
        slideRight.setTargetPosition(slideTarget);
    }

    private void resetSlideEncoder() {
        slideTarget = 0;
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setTargetPosition(slideTarget);
        slideRight.setTargetPosition(slideTarget);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(SLIDE_POWER);
        slideRight.setPower(SLIDE_POWER);
        slideMinSeen = 0;
        slideMaxSeen = 0;
    }

    private void updateSeenLimits() {
        if (slideMode) {
            int current = averageSlidePosition();
            slideMinSeen = Math.min(slideMinSeen, current);
            slideMaxSeen = Math.max(slideMaxSeen, current);
        } else {
            servoChannels[selectedServo].updateSeenLimits();
        }
    }

    private void showInitTelemetry() {
        telemetry.addLine("Mechanism Fine Tuner");
        telemetry.addLine("G2 Y: Servo/Slide mode");
        telemetry.addLine("G2 X: coarse/fine step");
        telemetry.addLine("G2 dpad left/right: select servo in Servo mode");
        telemetry.addLine("G2 dpad up/down or triggers: tune value");
        telemetry.addLine("G2 A/B: jump to recommended min/max");
        telemetry.addLine("G2 START: reset observed limits");
        telemetry.addLine("G2 BACK: reset slide encoder in Slide mode");
        telemetry.update();
    }

    private void showTelemetry() {
        telemetry.addLine("Mechanism Fine Tuner");
        telemetry.addData("Mode", slideMode ? "Slide encoder target" : "Servo position");
        telemetry.addData("Step", slowMode ? "Fine" : "Coarse");
        telemetry.addLine("Y mode | X step | UP/DOWN tune | LT/RT fast tune");
        telemetry.addLine("A min | B max | START reset observed limits");

        if (slideMode) {
            telemetry.addData("Slide target", slideTarget);
            telemetry.addData("Slide recommended min/max", "%d / %d", SLIDE_MIN_LIMIT, SLIDE_MAX_LIMIT);
            telemetry.addData("Slide observed min/max", "%d / %d", slideMinSeen, slideMaxSeen);
            telemetry.addData("SlideLeft current/target", "%d / %d", slideLeft.getCurrentPosition(), slideLeft.getTargetPosition());
            telemetry.addData("SlideRight current/target", "%d / %d", slideRight.getCurrentPosition(), slideRight.getTargetPosition());
            telemetry.addData("Slide power", "%.2f", SLIDE_POWER);
            telemetry.addLine("BACK resets slide encoder zero.");
        } else {
            ServoChannel selected = servoChannels[selectedServo];
            telemetry.addData("Selected", "%d/%d %s", selectedServo + 1, servoChannels.length, selected.name);
            telemetry.addData("Position", "%.3f", selected.position);
            telemetry.addData("Recommended min/max", "%.3f / %.3f", selected.recommendedMin, selected.recommendedMax);
            telemetry.addData("Observed min/max", "%.3f / %.3f", selected.minSeen, selected.maxSeen);
            telemetry.addData("Servo count", selected.servos.length);
            telemetry.addLine("DPAD left/right changes selected mechanism.");

            for (int i = 0; i < servoChannels.length; i++) {
                ServoChannel channel = servoChannels[i];
                telemetry.addData(channel.name, "%.3f  seen %.3f~%.3f", channel.position, channel.minSeen, channel.maxSeen);
            }
        }

        telemetry.update();
    }

    private double currentServoStep() {
        return slowMode ? SERVO_FINE_STEP : SERVO_STEP;
    }

    private int currentSlideStep() {
        return slowMode ? SLIDE_FINE_STEP : SLIDE_STEP;
    }

    private int averageSlidePosition() {
        return (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2;
    }

    private boolean pressedDpadUp() {
        return gamepad2.dpad_up && !previousDpadUp;
    }

    private boolean pressedDpadDown() {
        return gamepad2.dpad_down && !previousDpadDown;
    }

    private boolean pressedDpadLeft() {
        return gamepad2.dpad_left && !previousDpadLeft;
    }

    private boolean pressedDpadRight() {
        return gamepad2.dpad_right && !previousDpadRight;
    }

    private boolean pressedA() {
        return gamepad2.a && !previousA;
    }

    private boolean pressedB() {
        return gamepad2.b && !previousB;
    }

    private boolean pressedX() {
        return gamepad2.x && !previousX;
    }

    private boolean pressedY() {
        return gamepad2.y && !previousY;
    }

    private boolean pressedStart() {
        return gamepad2.start && !previousStart;
    }

    private boolean pressedBack() {
        return gamepad2.back && !previousBack;
    }

    private void rememberButtons() {
        previousDpadUp = gamepad2.dpad_up;
        previousDpadDown = gamepad2.dpad_down;
        previousDpadLeft = gamepad2.dpad_left;
        previousDpadRight = gamepad2.dpad_right;
        previousA = gamepad2.a;
        previousB = gamepad2.b;
        previousX = gamepad2.x;
        previousY = gamepad2.y;
        previousLeftBumper = gamepad2.left_bumper;
        previousRightBumper = gamepad2.right_bumper;
        previousStart = gamepad2.start;
        previousBack = gamepad2.back;
    }

    private double clip(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    private int clip(int value, int min, int max) {
        return Math.min(max, Math.max(min, value));
    }

    private class ServoChannel {
        private final String name;
        private final Servo[] servos;
        private final double recommendedMin;
        private final double recommendedMax;
        private double position;
        private double minSeen;
        private double maxSeen;

        private ServoChannel(String name, double startPosition, double recommendedMin, double recommendedMax, Servo... servos) {
            this.name = name;
            this.servos = servos;
            this.recommendedMin = recommendedMin;
            this.recommendedMax = recommendedMax;
            this.position = clip(startPosition, 0.0, 1.0);
            this.minSeen = this.position;
            this.maxSeen = this.position;
        }

        private void move(double delta) {
            set(position + delta);
        }

        private void set(double newPosition) {
            position = clip(newPosition, 0.0, 1.0);
            apply();
            updateSeenLimits();
        }

        private void apply() {
            for (Servo servo : servos) {
                servo.setPosition(position);
            }
        }

        private void updateSeenLimits() {
            minSeen = Math.min(minSeen, position);
            maxSeen = Math.max(maxSeen, position);
        }

        private void resetSeenLimits() {
            minSeen = position;
            maxSeen = position;
        }
    }
}
