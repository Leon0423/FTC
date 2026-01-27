package org.firstinspires.ftc.teamcode.TITAN;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static java.lang.Math.PI;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class Titan_Base extends LinearOpMode {
    public DcMotor FR, FL, BR, BL, intake, middle;
    public DcMotorEx  shooterR, shooterL;
    public Servo sonicL, sonicR;
    protected GoBildaPinpointDriver odo;

    // 射擊狀態機
    public enum ShootingState {
        IDLE,
        SPIN_UP,        // 飛輪加速
        READY,          // 飛輪穩定，準備射擊
        FEEDING,        // 推球中
        WAIT_BALL,      // 等待球到位
        DONE            // 完成
    }

    private ShootingState shootingState = ShootingState.IDLE;
    private int targetVelocity = 0;
    private int ballCount = 0;
    private int totalBalls = 3;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime stabilityTimer = new ElapsedTime();

    // 參數設定
    private int feedAmount = 500;
    private int retractAmount = 100;
    private int Vf = 1400;
    private int Vc = 1050;
    private int velocityTolerance = 20;
    private int stabilityTime = 50;  // 飛輪穩定時間 (ms)
    private int feedWaitTime = 100;  // 每顆球之間等待時間 (ms)

    public void init_hardware() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sonicR = hardwareMap.get(Servo.class, "sonicR");
        sonicL = hardwareMap.get(Servo.class, "sonicL");
        sonicL.setDirection(Servo.Direction.REVERSE);
        sonicR.setPosition(0);
        sonicL.setPosition(0);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        middle = hardwareMap.get(DcMotorEx.class, "middle");
        middle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middle.setTargetPosition(0);
        middle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, 168.0, MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(MM, 0, 0, AngleUnit.RADIANS, PI / 2);
        odo.setPosition(startingPosition);
    }

    public void moveRobot() {
        if (gamepad1.options) {
            odo.setPosition(new Pose2D(MM, 0, 0, AngleUnit.RADIANS, PI / 2));
        }

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((PI / 2) - heading);
        double sinAngle = Math.sin((PI / 2) - heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];
        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        FL.setPower(newWheelSpeeds[0]);
        FR.setPower(newWheelSpeeds[1]);
        BL.setPower(newWheelSpeeds[2]);
        BR.setPower(newWheelSpeeds[3]);
    }

    public void intake() {
        if (gamepad1.right_bumper) {
            intake.setPower(1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

    // 啟動遠距射擊
    public void startShootingf() {
        if (shootingState == ShootingState.IDLE) {
            initShooting(Vf);
        }
    }

    // 啟動近距射擊
    public void startShootingc() {
        if (shootingState == ShootingState.IDLE) {
            initShooting(Vc);
        }
    }

    // 初始化射擊
    private void initShooting(int velocity) {
        targetVelocity = velocity;
        ballCount = 0;

        // 先後退
        int pos = middle.getCurrentPosition();
        middle.setTargetPosition(pos - retractAmount);
        middle.setPower(1.0);

        shootingState = ShootingState.SPIN_UP;
        stateTimer.reset();
        stabilityTimer.reset();
    }

    // 主要更新方法 - 每個迴圈呼叫
    public void updateShooting() {
        switch (shootingState) {
            case IDLE:
                break;

            case SPIN_UP:
                // 啟動飛輪
                shooterL.setVelocity(targetVelocity);
                shooterR.setVelocity(targetVelocity);
                intake.setPower(1);

                // 檢查飛輪是否穩定
                if (isFlywheelStable()) {
                    if (stabilityTimer.milliseconds() >= stabilityTime) {
                        shootingState = ShootingState.READY;
                        stateTimer.reset();
                    }
                } else {
                    stabilityTimer.reset();
                }
                break;

            case READY:
                // 飛輪穩定，開始推第一顆球
                feedBall();
                shootingState = ShootingState.FEEDING;
                break;

            case FEEDING:
                // 等待推球完成
                if (!middle.isBusy()) {
                    ballCount++;

                    // 第三顆球啟動 sonic
                    if (ballCount == 3) {
                        double sonicPos = (targetVelocity == Vf) ? 0.4 : 0.35;
                        sonicR.setPosition(sonicPos);
                    }

                    // 檢查是否完成所有球
                    if (ballCount >= totalBalls) {
                        shootingState = ShootingState.DONE;
                        stateTimer.reset();
                    } else {
                        shootingState = ShootingState.WAIT_BALL;
                        stateTimer.reset();
                    }
                }
                break;

            case WAIT_BALL:
                // 等待下一顆球到位
                if (stateTimer.milliseconds() >= feedWaitTime) {
                    if (isFlywheelStable()) {
                        shootingState = ShootingState.READY;
                    } else {
                        // 飛輪不穩定，重新加速
                        shootingState = ShootingState.SPIN_UP;
                        stabilityTimer.reset();
                    }
                }
                break;

            case DONE:
                // 清理並返回 IDLE
                if (stateTimer.milliseconds() >= 500) {
                    sonicR.setPosition(0);
                    shooterL.setVelocity(0);
                    shooterR.setVelocity(0);
                    intake.setPower(0);
                    shootingState = ShootingState.IDLE;
                }
                break;
        }

        // 顯示狀態
        telemetry.addData("射擊狀態", shootingState);
        telemetry.addData("已射球數", "%d/%d", ballCount, totalBalls);
        telemetry.addData("飛輪速度", "L:%.0f R:%.0f",
                Math.abs(shooterL.getVelocity()),
                Math.abs(shooterR.getVelocity()));
    }

    // 檢查飛輪是否穩定
    private boolean isFlywheelStable() {
        double velL = Math.abs(shooterL.getVelocity());
        double velR = Math.abs(shooterR.getVelocity());

        boolean lStable = Math.abs(velL - targetVelocity) < velocityTolerance;
        boolean rStable = Math.abs(velR - targetVelocity) < velocityTolerance;

        return lStable && rStable;
    }

    // 推球
    private void feedBall() {
        int pos = middle.getCurrentPosition();
        middle.setTargetPosition(pos + feedAmount);
        middle.setPower(1.0);
    }

    // 停止射擊
    public void stopShooting() {
        shooterL.setVelocity(0);
        shooterR.setVelocity(0);
        intake.setPower(0);
        sonicR.setPosition(0);
        shootingState = ShootingState.IDLE;
    }
}