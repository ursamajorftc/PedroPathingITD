package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.002;
        ThreeWheelConstants.strafeTicksToInches = 0.0013;
        ThreeWheelConstants.turnTicksToInches = 0.0029;

        ThreeWheelConstants.leftY = 3.720472;
        ThreeWheelConstants.rightY = -3.720472;
        ThreeWheelConstants.strafeX = -7.1299213;

        ThreeWheelConstants.leftEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "leftBack";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightBack";

        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




