package pedroPathing.constants;

import com.qualcomm.robotcore.hardware.Servo;

public class RConstants {
	//wrist positions
	public static final double WRISTPOSITIONDOWN = 0;
	public static final double WRISTPOSITIONSTRAIGHT = 0.62;
	public static final double WRISTPOSITIONOUT = 1;

	//arm positions
	public static final double ARMPOSITIONDEPOSIT = 0.425;
	public static final double ARMPOSITIONHOVER = 0.815;
	public static final double ARMPOSITIONGRAB = 0.95;
	public static final int HIGHBASKET = 1100;
	public static final int LOWBASKET = 530;
	public static final int DOWNPOSITION = 0;

	//claw positions
	public static final double CLAWPOSITIONOPEN = 0.26;
	public static final double CLAWPOSITIONCLOSED = 0.48;

	//miscellaneous
	public boolean previousDpadDownState = false;
	public boolean previousDpadUpState = false;
	public boolean PreviousDpadLeftState = false;
	public boolean previousAState = false;
	public boolean previousIntakeState = false;
	public boolean sampleDistanceTriggered = false;
	public int state = 0; // Persistent state variable
	public long startTime = 0; // Persistent timer variable
	public static final float[] hsvValues = new float[3];
	public static final double INTAKESERVOPOSITION = 0;
}
