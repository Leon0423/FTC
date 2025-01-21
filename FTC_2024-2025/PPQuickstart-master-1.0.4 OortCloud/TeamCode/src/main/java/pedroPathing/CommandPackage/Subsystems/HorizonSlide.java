package pedroPathing.CommandPackage.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.CommandPackage.RobotConstants;

public class HorizonSlide extends SubsystemBase {
    // * 宣告硬體組件
    private final CRServo HSRight, HSLeft;
    private final AnalogInput HSRightEncoder;

    // * 建構子 - 初始化硬體
    public HorizonSlide(final HardwareMap hardwareMap) {
        // 取得馬達參考
        // * CRServo
        HSRight = hardwareMap.get(CRServo.class, "HSRight");
        HSLeft = hardwareMap.get(CRServo.class, "HSLeft");
        HSRight.setDirection(CRServo.Direction.REVERSE);
        HSLeft.setDirection(CRServo.Direction.FORWARD);

        // * Analog Input
        HSRightEncoder = hardwareMap.get(AnalogInput.class, "HSRightEncoder");
    }

    public void setSlidePower(double power) {
        HSRight.setPower(power);
        HSLeft.setPower(power);
    }

    public double getSlideVoltage() {
        return HSRightEncoder.getVoltage();
    }
    public double getSlideEncoderPosition() {
        return HSRightEncoder.getVoltage() / 3.3 * 360;
    }

}
