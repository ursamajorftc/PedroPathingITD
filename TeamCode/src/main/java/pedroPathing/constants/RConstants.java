package pedroPathing.constants;

import com.qualcomm.robotcore.hardware.Servo;

public class RConstants {
	//wrist positions
	public static final double WRISTPOSITIONDOWN = 0.07;
	public static final double WRISTPOSITIONSTRAIGHT = 0.62;
	public static final double WRISTPOSITIONOUT = 1;

	//arm positions
	public static final double ARMPOSITIONDEPOSIT = 0.475;
	public static final double ARMPOSITIONHOVER = 0.855;
	public static final double ARMPOSITIONGRAB = 0.97;
	public static final int HIGHBASKET = 1100;
	public static final int LOWBASKET = 530;
	public static final int DOWNPOSITION = 0;

	//claw positions
	public static final double CLAWPOSITIONOPEN = 0.26;
	public static final double CLAWPOSITIONCLOSED = 0.46;

	//spec Claw positions
	public static final double SPECCLAWOPEN = 0.3;
	public static final double SPECCLAWCLOSED = 0.55;

	// spec Arm positions

	public static final int  SPECARMUP = 420;
	public static final int SPECARMDOWN = 0;



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
	//intake
	public static final double INTAKESERVOPOSITION = 0;
	public static final double RIGHTINTAKEUP = 0.695;
	public static final double LEFTINTAKEUP = 0.32;
	public static final double RIGHTINTAKEDOWN = 0.45;
	public static final double LEFTINTAKEDOWN = 0.54;

	public enum RobotState {
		IDLE,
		CLOSE_CLAW,
		MOVE_ARM,
		MOVE_WRIST,
		OPEN_CLAW,
		COMPLETE
	}

	public enum DepositState {
		UP,
		DOWN
	}

	public enum IntakeState {
		INTAKE0,
		INTAKE300,
		INTAKEDOWN,
		INTAKE880,
		INTAKESTOP,
		INTAKE350
	}

}


