package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = -0.00048;
        ThreeWheelConstants.strafeTicksToInches =  -0.000465;
        ThreeWheelConstants.turnTicksToInches = -0.0005;
        ThreeWheelConstants.leftY = 104.2/25.4;
        ThreeWheelConstants.rightY = -104.2/25.4;
        ThreeWheelConstants.strafeX = -152/25.4;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "FL";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "BL";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "FR";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




