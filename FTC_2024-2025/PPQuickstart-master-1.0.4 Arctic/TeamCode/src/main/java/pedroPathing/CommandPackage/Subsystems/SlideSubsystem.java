package pedroPathing.CommandPackage.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.CommandPackage.RobotConstants;

public class SlideSubsystem extends SubsystemBase {

    private final DcMotorEx SlideLeftFront, SlideLeftBack, SlideRightFront, SlideRightBack;

    public SlideSubsystem(final HardwareMap hardwareMap) {
        SlideLeftFront = hardwareMap.get(DcMotorEx.class, "SlideLF");
        SlideLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideLeftBack = hardwareMap.get(DcMotorEx.class, "SlideLB");
        SlideLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        SlideRightFront = hardwareMap.get(DcMotorEx.class, "SlideRF");
        SlideRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideRightBack = hardwareMap.get(DcMotorEx.class, "SlideRB");
        SlideRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //------------------------------Slide------------------------------//

    public void setSlidePower(double power) {
        SlideLeftFront.setPower(power);
        SlideLeftBack.setPower(power);
        SlideRightFront.setPower(power);
        SlideRightBack.setPower(power);
    }

    public int getSlidePosition() {
        return SlideRightBack.getCurrentPosition();
    }

    public void setDefaultPosition() {
        if(getSlidePosition() > RobotConstants.DefaultSlidePosition) {
            setSlidePower(-RobotConstants.SlidePower);
        } else if(getSlidePosition() < 0) {
            setSlidePower(RobotConstants.SlidePower);
        } else {
            setSlidePower(RobotConstants.SlideStopPower);
        }
    }

}