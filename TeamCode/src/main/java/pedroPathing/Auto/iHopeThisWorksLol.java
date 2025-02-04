package pedroPathing.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.constants.RConstants;
import pedroPathing.pid.PIDController;


// This is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking in the submersible zone.

@Autonomous(name = "i hope this works", group = "Examples")
public class iHopeThisWorksLol extends OpMode {



	//region Initializations
	private FtcDashboard dashboard;

	// Intake
	private CRServo intakeCRSLeft = null;
	private CRServo intakeCRSRight = null;
	private Servo intakeServoLeft = null;
	private Servo intakeServoRight = null;
	private Servo lockServo = null;
	private DcMotor intakeDrive = null;

	// Deposit
	private Servo clawServo = null;
	private Servo wristServo = null;
	private Servo armServo = null;

	private DcMotor outmoto1 = null;
	private DcMotor outmoto2 = null;

	// Utility
	ElapsedTime timer = new ElapsedTime();
	private Follower follower;
	private Timer pathTimer, actionTimer, opmodeTimer;
	private PIDController backPid = new PIDController(0.04, 0, 0.005);
	private boolean previousAState = false;
	private double power;
	private boolean scored = false;
	private boolean safe = false;
	private DepositState depositState = DepositState.IDLE;
	private IntakeState intakeState = IntakeState.INTAKESTART;
	private long stateStartTime = 0;
	private boolean downState = false;
	//endregion
	//region Paths
	/**
	 * This is the variable where we store the state of our auto.
	 * It is used by the pathUpdate method.
	 */
	private int pathState;

	/* Create and Define Poses + Paths
	 * Poses are built with three constructors: x, y, and heading (in Radians).
	 * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
	 * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
	 * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
	 * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
	/***/

	 // Start Pose of our robot

	private final Pose startPose = new Pose(11, 114, Math.toRadians(270));

	/**
	 * Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle.
	 */
	private final Pose scorePose = new Pose(15.459, 127.365, Math.toRadians(315));

	/**
	 * Lowest (First) Sample from the Spike Mark
	 */
	private final Pose pickup1Pose = new Pose(21, 122.5, Math.toRadians(0));

	/**
	 * Middle (Second) Sample from the Spike Mark
	 */
	private final Pose pickup2Pose = new Pose(21, 130, Math.toRadians(0));

	/**
	 * Highest (Third) Sample from the Spike Mark
	 */
	private final Pose pickup3Pose = new Pose(21, 129, Math.toRadians(30));

	/**
	 * Park Pose for our robot, after we do all of the scoring.
	 */
	private final Pose parkPose = new Pose(55, 114, Math.toRadians(270));

	/**
	 * Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
	 * The Robot will not go to this pose, it is used a control point for our bezier curve.
	 */
	private final Pose parkControlPose = new Pose(55, 120, Math.toRadians(90));

	/* These are our Paths and PathChains that we will define in buildPaths() */
	private Path scorePreload, park;
	private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

	/**
	 * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
	 * It is necessary to do this so that all the paths are built before the auto starts.
	 **/
	//endregion

	// This method is called once at the init of the OpMode.
	@Override
	public void init() {


		intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
		intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
		intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
		intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
		lockServo = hardwareMap.get(Servo.class, "lockServo");
		intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");

		// Deposit
		clawServo = hardwareMap.get(Servo.class, "clawServo");
		wristServo = hardwareMap.get(Servo.class, "wristServo");
		armServo = hardwareMap.get(Servo.class, "armServo");
		outmoto1 = hardwareMap.get(DcMotorEx.class, "outmoto1");
		outmoto2 = hardwareMap.get(DcMotorEx.class, "outmoto2");

		armServo.setPosition(RConstants.ARMPOSITIONDEPOSIT);
		wristServo.setPosition(RConstants.WRISTPOSITIONSTRAIGHT);
		clawServo.setPosition(RConstants.CLAWPOSITIONCLOSED);

		pathTimer = new Timer();
		opmodeTimer = new Timer();
		opmodeTimer.resetTimer();

		Constants.setConstants(FConstants.class, LConstants.class);
		follower = new Follower(hardwareMap);
		follower.setStartingPose(startPose);



		buildPaths();

//		intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//		outmoto1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//		outmoto2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//		outmoto1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//		outmoto2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
	}
	public void buildPaths() {

		/* There are two major types of paths components: BezierCurves and BezierLines.
		 *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
		 *    - Control points manipulate the curve between the start and end points.
		 *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
		 *    * BezierLines are straight, and require 2 points. There are the start and end points.
		 * Paths have can have heading interpolation: Constant, Linear, or Tangential
		 *    * Linear heading interpolation:
		 *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
		 *    * Constant Heading Interpolation:
		 *    - Pedro will maintain one heading throughout the entire path.
		 *    * Tangential Heading Interpolation:
		 *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
		 * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
		 * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

		/* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
		scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
		scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

		/* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
		grabPickup1 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
				.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
				.build();

		/* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
		scorePickup1 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
				.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
				.build();

		/* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
		grabPickup2 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
				.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
				.build();

		/* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
		scorePickup2 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
				.setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
				.build();

		/* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
		grabPickup3 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
				.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
				.build();

		/* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
		scorePickup3 = follower.pathBuilder()
				.addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
				.setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
				.build();

		/* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
		park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
		park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
	}

	/**
	 * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
	 * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
	 * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
	 */
	public void autonomousPathUpdate() {
		switch (pathState) {
			case 0:

				follower.followPath(scorePreload,true);
				backPid.setTargetPosition(1150);


					setPathState(1);




//				backPid.setTargetPosition(1150);

				break;
			case 1:
				if (!scored && (outmoto1.getCurrentPosition() > 1150)){
					armServo.setPosition(RConstants.ARMPOSITIONDEPOSIT);
					wristServo.setPosition(RConstants.WRISTPOSITIONOUT);
					clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
					downState = true;
					scored = true;
				}





                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"`
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
				if (!follower.isBusy() && safe) {
					/* Score Preload */
//					scoreSample();
					/* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
					follower.followPath(grabPickup1, true);


//					setPathState(2);
				}
				break;
			case 2:
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
				if (!follower.isBusy()) {

					/* Grab Sample */
				//	grabSample(350);
					/* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
					follower.followPath(scorePickup1, true);
					setPathState(3);
				}
				break;
			case 3:
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
				if (!follower.isBusy()) {
					/* Score Sample */
				//	scoreSample();
					/* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
					follower.followPath(grabPickup2, true);
					setPathState(4);
				}
				break;
			case 4:
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
				if (!follower.isBusy()) {
					/* Grab Sample */
					//grabSample(350);
					/* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
					follower.followPath(scorePickup2, true);
					setPathState(5);
				}
				break;
			case 5:
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
				if (!follower.isBusy()) {
					/* Score Sample */
				//	scoreSample();
					/* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
					follower.followPath(grabPickup3, true);
					setPathState(6);
				}
				break;
			case 6:
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
				if (!follower.isBusy()) {
					/* Grab Sample */
					//grabSample(425);
					/* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
					follower.followPath(scorePickup3, true);
					setPathState(7);
				}
				break;
			case 7:
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
				if (!follower.isBusy()) {
					/* Score Sample */
					//scoreSample();
					/* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
					follower.followPath(park, true);
					setPathState(8);
				}
				break;
			case 8:
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
				if (!follower.isBusy()) {
					/* Level 1 Ascent */

					/* Set the state to a Case we won't use or define, so it just stops running an new paths */
					setPathState(-1);
				}
				break;
		}
	}


	// These change the states of the paths and actions
	// It will also reset the timers of the individual switches
	public void setPathState(int pState) {
		pathState = pState;
		pathTimer.resetTimer();
	}


	// This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
	@Override
	public void loop() {

		// These loop the movements of the robot
		follower.update();
		autonomousPathUpdate();
		updateArmRetracty();

		// Feedback to Driver Hub
		telemetry.addData("path state", pathState);
		telemetry.addData("x", follower.getPose().getX());
		telemetry.addData("y", follower.getPose().getY());
		telemetry.addData("heading", follower.getPose().getHeading());
		telemetry.addData("outmoto1 position", outmoto1.getCurrentPosition());
		telemetry.update();

		power = backPid.getPower(outmoto1.getCurrentPosition());
		if (outmoto1.getCurrentPosition() > backPid.getTargetPosition()) {
			outmoto1.setPower(-0.2);
			outmoto2.setPower(0.2);
		} else {
			outmoto1.setPower(power);
			outmoto2.setPower(-power);
		}

	}

	private RConstants.RobotState currentState = RConstants.RobotState.IDLE;



	public void updateArmRetracty() {

		// Get the current time in milliseconds
		long currentTime = System.currentTimeMillis();


		switch (currentState) {
			case IDLE:
				if (downState) {
					stateStartTime = currentTime; // Record the time

				}
				if ((currentTime - stateStartTime >= 200) && downState) {
					// Transition to CLOSE_CLAW state
					clawServo.setPosition(RConstants.CLAWPOSITIONCLOSED);
					stateStartTime = currentTime; // Record the time
					currentState = RConstants.RobotState.CLOSE_CLAW;
				}
				break;

			case CLOSE_CLAW:
				if (currentTime - stateStartTime >= 200) { // Wait 200ms
					armServo.setPosition(0.475);
					stateStartTime = currentTime;
					currentState = RConstants.RobotState.MOVE_ARM;
				}
				break;

			case MOVE_ARM:
				if (currentTime - stateStartTime >= 200) { // Wait 200ms
					wristServo.setPosition(RConstants.WRISTPOSITIONDOWN);
					stateStartTime = currentTime;
					currentState = RConstants.RobotState.MOVE_WRIST;
				}
				break;

			case MOVE_WRIST:
				if (currentTime - stateStartTime >= 200) { // Wait 200ms
					clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
					stateStartTime = currentTime;
					currentState = RConstants.RobotState.OPEN_CLAW;
				}
				break;

			case OPEN_CLAW:
				if (currentTime - stateStartTime >= 100) { // Wait 200ms
					backPid.setTargetPosition(0);
					telemetry.addData("yippee", gamepad1.a);
					stateStartTime = currentTime;
					currentState = RConstants.RobotState.COMPLETE;
				}
				break;

			case COMPLETE:
				if (currentTime - stateStartTime > 200) {
					currentState = RConstants.RobotState.IDLE;
					intakeDrive.setTargetPosition(0);
					downState = false;
				}
				// All actions complete; stay idle or transition as needed
				break;
		}
	}



	//region Intake Methods
//	public void setIntakePos(int distance) {
//		intakeDrive.setTargetPosition(distance);
//		intakeDrive.setPower(1);
//	}
//
//	public double getIntakePos() {
//		return intakeDrive.getCurrentPosition();
//	}
//
//	public void intakeDown() {
//		intakeServoLeft.setPosition(RConstants.LEFTINTAKEDOWN);
//		intakeServoRight.setPosition(RConstants.RIGHTINTAKEDOWN);
//	}
//
//	public void intakeUp() {
//		intakeServoLeft.setPosition(RConstants.LEFTINTAKEUP);
//		intakeServoRight.setPosition(RConstants.RIGHTINTAKEUP);
//	}
//	public void spinIntake(){
//		intakeCRSLeft.setPower(-1);
//		intakeCRSRight.setPower(1);
//	}
//	public void spinIntake(double power){
//		intakeCRSLeft.setPower(-power);
//		intakeCRSRight.setPower(power);
//	}
//
//	public void openLock() {
//		lockServo.setPosition(0);
//	}
//
//	public void closeLock() {
//		lockServo.setPosition(0.3);
//	}
	public enum IntakeState {
		INTAKESTART,
		INTAKEOUT,
		INTAKEDOWN,
		INTAKEUP
	}


//	public void grabSample(int distance) {
//		switch (intakeState){
//			case INTAKESTART:
//				setIntakePos(distance);
//				intakeState = IntakeState.INTAKEOUT;
//
//			case INTAKEOUT:
//				if (intakeDrive.getCurrentPosition() > 250) {
//					intakeDown();
//					spinIntake();
//					closeLock();
//					opmodeTimer.resetTimer();
//					intakeState = IntakeState.INTAKEDOWN;
//				}
//				break;
//			case INTAKEDOWN:
//				if (opmodeTimer.getElapsedTimeSeconds() > 2) {
//					intakeUp();
//					spinIntake(0.2);
//					opmodeTimer.resetTimer();
//					intakeState = IntakeState.INTAKEUP;
//				}
//				break;
//			case INTAKEUP:
//				if (opmodeTimer.getElapsedTimeSeconds() > 0.5) {
//					setIntakePos(0);
//					openLock();
//					intakeState = IntakeState.INTAKESTART;
//				}
//				break;
//		}
//	}
	//endregion
	//region Deposit Methods
//	public void closeClaw() {
//		clawServo.setPosition(RConstants.CLAWPOSITIONCLOSED);
//	}
//
//	public void openClaw() {
//		clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
//	}
//
//	public void wristDown() {
//		wristServo.setPosition(RConstants.WRISTPOSITIONDOWN);
//	}
//
//	public void wristBack() {
//		wristServo.setPosition(RConstants.WRISTPOSITIONOUT);
//	}
//
//	public void armDeposit() {
//		armServo.setPosition(RConstants.ARMPOSITIONDEPOSIT);
//	}
//
//	public void armHover() {
//		armServo.setPosition(RConstants.ARMPOSITIONHOVER);
//	}
//
//	public void armGrab() {
//		armServo.setPosition(RConstants.ARMPOSITIONGRAB);
//	}
	enum DepositState {
		IDLE,
		CLOSE_CLAW,
		MOVE_ARM,
		MOVE_WRIST,
		RESET,
		COMPLETE
	}

	//endregion

	// This method is called continuously after Init while waiting for "play".
	@Override
	public void init_loop() {
	}

	// This method is called once at the start of the OpMode.
	// It runs all the setup actions, including building paths and starting the path system
	@Override
	public void start() {
		opmodeTimer.resetTimer();
		setPathState(0);
	}

	// We do not use this because everything should automatically disable
	@Override
	public void stop() {
	}
}