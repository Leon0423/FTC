package pedroPathing.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.config.RobotConstants;


/** This is a subsystem, for the claw of our robot
 * Here we make methods to manipulate the servos
 * We also import RobotConstants to get the positions of the servos.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

public class SlideSubsystem {

    private DcMotorEx SlideLeftFront, SlideLeftBack, SlideRightFront, SlideRightBack;

    /** This is the constructor for the subsystem, it maps the servos to the hardwareMap.
     * The device names should align with the configuration names on the driver hub.
     * To use this subsystem, we have to import this file, declare the subsystem (private ClawSubsystem claw;),
     * and then call the below constructor in the init() method. */

    public SlideSubsystem(HardwareMap hardwareMap) {
        SlideLeftFront = hardwareMap.get(DcMotorEx.class, "SlideLeftFront");
        SlideLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideLeftBack = hardwareMap.get(DcMotorEx.class, "SlideLeftBack");
        SlideLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideRightFront = hardwareMap.get(DcMotorEx.class, "SlideRightFront");
        SlideRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideRightBack = hardwareMap.get(DcMotorEx.class, "SlideRightBack");
        SlideRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // 2. 狀態監控：檢查是否到達目標位置
        if (isMoving && Math.abs(motor.getCurrentPosition() - targetPosition) < 10) {
            motor.setPower(0);
            isMoving = false;
        }
    }

     */

    //------------------------------Slide------------------------------//

    public void Slide(double power) {
        SlideLeftFront.setPower(power);
        SlideLeftBack.setPower(power);
        SlideRightFront.setPower(power);
        SlideRightBack.setPower(power);
    }


    /** This is the closeClaw method, it sets the grab to the closed position defined in RobotConstants. */

    public void slidesUp(double power) {
        Slide(power);
    }

    public void slidesDown(double power) {
        Slide(-power);
    }


}