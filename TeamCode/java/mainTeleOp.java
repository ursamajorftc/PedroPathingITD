//region Imports
package pedroPathing.tuners_tests;

import android.view.View;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
//endregion

@TeleOp(name = "mainTeleOp", group = "Linear OpMode")
public class mainTeleOp extends OpMode {
	//region Initializations
	private Follower follower;
	private FtcDashboard dashboard;
	private ElapsedTime runtime = new ElapsedTime();
	private CRServo intakeCRSLeft = null;
	private CRServo intakeCRSRight = null;
	private Servo intakeServoLeft = null;
	private Servo lockServo = null;
	private DcMotor intakeDrive = null;
	private Servo clawServo = null;
	private Servo wristServo = null;
	private Servo armServo = null;
	private DcMotor outmoto1 = null;
	private DcMotor outmoto2 = null;
	private static final Pose startPose = new Pose(61.302,96.9,270);
	PIDController pid = new PIDController(0.04, 0, 0.05);
	NormalizedColorSensor colorSensor;
	NormalizedColorSensor sampleDistance;
	View relativeLayout;
	//endregion
	//region Constants
	//wrist positions
	private static final double WRISTPOSITIONDOWN = 0;
	private static final double WRISTPOSITIONSTRAIGHT = 0.62;
	private static final double WRISTPOSITIONOUT = 1;

	//arm positions
	private static final double ARMPOSITIONDEPOSIT = 0.425;
	private static final double ARMPOSITIONHOVER = 0.815;
	private static final double ARMPOSITIONGRAB = 0.95;
	private static final int HIGHBASKET = 1100;
	private static final int LOWBASKET = 530;
	private static final int DOWNPOSITION = 0;

	//claw positions
	private static final double CLAWPOSITIONOPEN = 0.26;
	private static final double CLAWPOSITIONCLOSED = 0.48;

	//miscellaneous
	private boolean previousDpadDownState = false;
	private boolean previousDpadUpState = false;
	private boolean PreviousDpadLeftState = false;
	private boolean previousAState = false;
	private boolean previousIntakeState = false;
	private Servo intakeServoRight = null;
	private boolean sampleDistanceTriggered = false;
	private int state = 0; // Persistent state variable
	private long startTime = 0; // Persistent timer variable
	private static final float[] hsvValues = new float[3];
	private static final double INTAKESERVOPOSITION = 0;
	//endregion

	/** This method is called once when init is played, it initializes the follower **/
	@Override
	public void init() {
		Constants.setConstants(FConstants.class, LConstants.class);
		follower = new Follower(hardwareMap);
		follower.setStartingPose(startPose);
	}

	/** This method is called continuously after Init while waiting to be started. **/
	@Override
	public void init_loop() {
	}

	/** This method is called once at the start of the OpMode. **/
	@Override
	public void start() {
		follower.startTeleopDrive();
	}

	/** This is the main loop of the opmode and runs continuously after play **/
	@Override
	public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

		follower.setTeleOpMovementVectors(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, true);
		follower.update();

		/* Telemetry Outputs of our Follower */
		telemetry.addData("X", follower.getPose().getX());
		telemetry.addData("Y", follower.getPose().getY());
		telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

		/* Update Telemetry to the Driver Hub */
		telemetry.update();

	}

	/** We do not use this because everything automatically should disable **/
	@Override
	public void stop() {
	}
}