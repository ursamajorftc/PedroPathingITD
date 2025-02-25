package pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.constants.RConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "5 Specimens", group = "Examples")
public class iHopeSpecsWorkLol extends OpMode {

	private Follower follower;
	private Timer pathTimer, opmodeTimer, scoreTimer;
	//    private DcMotor intakeDrive = null;
	private DcMotor specDrive = null;
	private Servo specServo = null;
	TouchSensor rightTouchSensor;
	TouchSensor leftTouchSensor;
	private DcMotor intakeDrive;

	/**
	 * This is the variable where we store the state of our auto.
	 * It is used by the pathUpdate method.
	 */
	private int pathState;


	private final Pose startPose = new Pose(8.401, 68, Math.toRadians(180));

	//push samples
	private final Pose startPush1 = new Pose(38, 47, Math.toRadians(180));
	private final Pose endPush1 = new Pose(22, 42, Math.toRadians(180));
	private final Pose endPush2 = new Pose(22, 32, Math.toRadians(185));
	private final Pose startPush3 = new Pose(58, 32, Math.toRadians(185));
	private final Pose endPush3 = new Pose(23.5, 28, Math.toRadians(185));

	//grab specs
	private final Pose grabPose2 = new Pose(9.87, 37, Math.toRadians(185));
	private final Pose grabPose3 = new Pose(6.2, 36, Math.toRadians(180));
	private final Pose grabPose4 = new Pose(5.9, 36, Math.toRadians(180));
	private final Pose grabPose5 = new Pose(5, 36, Math.toRadians(180));

	//score specs
	private final Pose scorePose1 = new Pose(29.5, 72, Math.toRadians(180));
	private final Pose scorePose2 = new Pose(29.5, 70, Math.toRadians(180));
	private final Pose scorePose3 = new Pose(29.5, 68.5, Math.toRadians(180));
	private final Pose scorePose4 = new Pose(29.5, 67, Math.toRadians(180));
	private final Pose scorePose5 = new Pose(29.5, 66, Math.toRadians(180));
	private final Pose scoreControl = new Pose(22, 70);


	/* These are our Paths and PathChains that we will define in buildPaths() */
//    private Path scorePreload;
	private PathChain scorePreload, pushSamples, score2, grab3, score3, grab4, score4, grab5, score5;

	/**
	 * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
	 * It is necessary to do this so that all the paths are built before the auto starts.
	 **/
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

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

		scorePreload = follower.pathBuilder()
				.addPath(
						new BezierLine(
								new Point(startPose),
								new Point(scorePose1)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		pushSamples = follower.pathBuilder()
				// get to point before pushing sample 1
				.addPath(
						new BezierCurve(
								new Point(scorePose1),
								new Point(19, 43, Point.CARTESIAN),
								new Point(startPush1)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				// push sample 1
				.addPath(
						new BezierCurve(
								new Point(startPush1),
								new Point(55, 36, Point.CARTESIAN),
								new Point(endPush1)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				//push 2
				.addPath(
						new BezierCurve(
								new Point(endPush1),
								new Point(65, 39.5, Point.CARTESIAN),
								new Point(endPush2)
						)
				).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(185))
				// get to point before pushing 3
				.addPath(
						new BezierCurve(
								new Point(endPush2),
								new Point(59, 38, Point.CARTESIAN),
								new Point(startPush3)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(185))
				.setPathEndTValueConstraint(0.2)
				//push 3
				.addPath(
						new BezierLine(
								new Point(startPush3),
								new Point(endPush3)
						)
				).setConstantHeadingInterpolation(Math.toRadians(180))
				.addPath(
						new BezierLine(
								new Point(endPush3),
								new Point(grabPose2)
						)
				).setConstantHeadingInterpolation(Math.toRadians(185))
				.setZeroPowerAccelerationMultiplier(0.2)
				.build();
		score2 = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Point(grabPose2),
								new Point(scoreControl),
								new Point(scorePose2)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.setZeroPowerAccelerationMultiplier(0.2)
				.build();
		grab3 = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Point(scorePose2),
								new Point(scoreControl),
								new Point(grabPose3)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		score3 = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Point(grabPose3),
								new Point(scoreControl),
								new Point(scorePose3)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.setZeroPowerAccelerationMultiplier(0.2)
				.build();
		grab4 = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Point(scorePose3),
								new Point(scoreControl),
								new Point(grabPose4)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		score4 = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Point(grabPose4),
								new Point(scoreControl),
								new Point(scorePose4)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.setZeroPowerAccelerationMultiplier(0.2)
				.build();
		grab5 = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Point(scorePose4),
								new Point(scoreControl),
								new Point(grabPose5)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		score5 = follower.pathBuilder()
				.addPath(
						new BezierCurve(
								new Point(grabPose5),
								new Point(scoreControl),
								new Point(scorePose5)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.setZeroPowerAccelerationMultiplier(0.2)
				.build();


	}

	/**
	 * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
	 * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
	 * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
	 */
	public boolean isSpecScore = false;
	public boolean isSpecGrabbed = false;

	public void autonomousPathUpdate() {
		switch (pathState) {
			case 0:
				follower.followPath(scorePreload);
				specDrive.setTargetPosition(RConstants.SPECARMUP);
				setPathState(1);
				break;
			case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

				/* This case checks the robot's position and will wait until the robot position  is close (1 inch away) from the scorePose1's position */
				if ((rightTouchSensor.isPressed() && leftTouchSensor.isPressed() || !follower.isBusy()) && !isSpecScore) {
//
					/* Score Preload */
					specScore = true;
					isSpecScore = true;


					/* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
				}

				if ((specDrive.getCurrentPosition() < 200) && isSpecScore) {
					follower.followPath(pushSamples, true);
					setPathState(2);
				}
				break;
			case 2:
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the endPush1's position */
				if ((!follower.isBusy() || (follower.getPose().getX() < grabPose2.getX())) && !isSpecGrabbed) {

					specGrabbed = false;
					isSpecGrabbed = true;
					/* Grab Sample */
					/* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
				}
//
				if ((specServo.getPosition() == RConstants.SPECCLAWCLOSED) && (specDrive.getCurrentPosition() > 200)) {
					follower.followPath(score2, 1, true);
					scoreTimer.resetTimer();
					setPathState(3);
					isSpecScore = false;
				}
				break;
			case 3:
				if ((rightTouchSensor.isPressed() && leftTouchSensor.isPressed()) && !isSpecScore) {
//
					/* Score Preload */
					specScore = true;
					isSpecScore = true;
					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));

					telemetry.addData("specScore", specScore);

					/* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
				} else if ((rightTouchSensor.isPressed() ^ leftTouchSensor.isPressed()) && scoreTimer.getElapsedTimeSeconds() >= 2.5) {
					specScore = true;
					isSpecScore = true;
					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
				}

//				else if (scoreTimer.getElapsedTimeSeconds() > 3){
//					specScore = true;
//					isSpecScore = true;
//					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
//				}

				if ((specDrive.getCurrentPosition() < 200) && isSpecScore) {
					follower.followPath(grab3, true);
					setPathState(7);
					isSpecGrabbed = false;
				}
				/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose1's position */

				break;
			case 7:
				if ((!follower.isBusy() || (follower.getPose().getX() < grabPose2.getX())) && !isSpecGrabbed) {

					specGrabbed = false;
					isSpecGrabbed = true;
					/* Grab Sample */
					/* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
				}
//
				if ((specServo.getPosition() == RConstants.SPECCLAWCLOSED) && (specDrive.getCurrentPosition() > 200)) {
					follower.followPath(score3,1, true);
					scoreTimer.resetTimer();
					setPathState(8);
					isSpecScore = false;
				}
				break;
			case 8:

				if ((rightTouchSensor.isPressed() && leftTouchSensor.isPressed()) && !isSpecScore) {
//
					/* Score Preload */
					specScore = true;
					isSpecScore = true;
					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));


					telemetry.addData("specScore", specScore);

					/* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
				} else if ((rightTouchSensor.isPressed() ^ leftTouchSensor.isPressed()) && scoreTimer.getElapsedTimeSeconds() >= 2.5) {
					specScore = true;
					isSpecScore = true;
					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
				}
//				else if (scoreTimer.getElapsedTimeSeconds() > 3){
//					specScore = true;
//					isSpecScore = true;
//					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
//				}

				if ((specDrive.getCurrentPosition() < 200) && isSpecScore) {
					follower.followPath(grab4, true);
					setPathState(9);
					isSpecGrabbed = false;

				}
				break;
			case 9:
				if ((!follower.isBusy() || (follower.getPose().getX() < grabPose2.getX())) && !isSpecGrabbed) {

					specGrabbed = false;
					isSpecGrabbed = true;
					/* Grab Sample */
					/* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
				}
//
				if ((specServo.getPosition() == RConstants.SPECCLAWCLOSED) && (specDrive.getCurrentPosition() > 200)) {
					follower.followPath(score4, 1, true);
					scoreTimer.resetTimer();
					setPathState(10);
					isSpecScore = false;
				}

				break;
			case 10:
				if ((rightTouchSensor.isPressed() && leftTouchSensor.isPressed()) && !isSpecScore) {
//
					/* Score Preload */
					specScore = true;
					isSpecScore = true;
					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));

					telemetry.addData("specScore", specScore);

					/* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
				} else if ((rightTouchSensor.isPressed() ^ leftTouchSensor.isPressed()) && scoreTimer.getElapsedTimeSeconds() >= 2.5) {
					specScore = true;
					isSpecScore = true;
					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
				}
//				else if (scoreTimer.getElapsedTimeSeconds() > 3){
//					specScore = true;
//					isSpecScore = true;
//					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
//				}

				if ((specDrive.getCurrentPosition() < 200) && isSpecScore) {
					follower.followPath(grab5, true);
					setPathState(11);
					isSpecGrabbed = false;

				}

				break;
			case 11:
				if ((!follower.isBusy() || ((follower.getPose().getX() - 0.1) < grabPose2.getX())) && !isSpecGrabbed) {

					specGrabbed = false;
					isSpecGrabbed = true;
					/* Grab Sample */
					/* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
				}
//
				if ((specServo.getPosition() == RConstants.SPECCLAWCLOSED) && (specDrive.getCurrentPosition() > 200)) {
					follower.followPath(score5, 1, true);
					scoreTimer.resetTimer();
					setPathState(12);
					isSpecScore = false;
				}

				break;
			case 12:
				if ((rightTouchSensor.isPressed() && leftTouchSensor.isPressed()) && !isSpecScore) {
//
					/* Score Preload */
					specScore = true;
					isSpecScore = true;
					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));

					telemetry.addData("specScore", specScore);

					/* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
				} else if ((rightTouchSensor.isPressed() ^ leftTouchSensor.isPressed()) && scoreTimer.getElapsedTimeSeconds() >= 2.5) {
					specScore = true;
					isSpecScore = true;
					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
				}
//				else if (scoreTimer.getElapsedTimeSeconds() > 3){
//					specScore = true;
//					isSpecScore = true;
//					follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
//				}

				if ((specDrive.getCurrentPosition() < 200) && isSpecScore) {

					setPathState(-1);
				}


				break;
		}
	}

	public boolean specScore = false;
	public boolean specGrabbed = false;
	public int specState = 1;
	public long specTimeStart = 0;

	public void SpecScore() {
		if (specScore) {
			specDrive.setTargetPosition(0);
			specDrive.setPower(0.7);
		}

		if (specScore && (specDrive.getTargetPosition() == 0) && specDrive.getCurrentPosition() < 320) {
			specServo.setPosition(0.3);
			specScore = false;
		}
	}

	public void SpecGrab() {
		if (!specGrabbed) {
			switch (specState) {
				case 1:
					specServo.setPosition(RConstants.SPECCLAWCLOSED);
					specState = 2;
					specTimeStart = System.currentTimeMillis();
					break;
				case 2:
					if ((System.currentTimeMillis() - specTimeStart) > 900) {
						specDrive.setTargetPosition(RConstants.SPECARMUP);
						specDrive.setPower(1);
						specState = 1;
						specGrabbed = true;
					}
			}
		}


	}

	public boolean XBUFFER(Pose robopose, double xbuffer) {

		return (Math.abs(robopose.getX() - follower.getPose().getX()) < xbuffer);
	}

	/**
	 * These change the states of the paths and actions
	 * It will also reset the timers of the individual switches
	 **/
	public void setPathState(int pState) {
		pathState = pState;
		pathTimer.resetTimer();
	}

	/**
	 * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
	 **/
	@Override
	public void loop() {

		// These loop the movements of the robot

		follower.update();
		autonomousPathUpdate();
		SpecScore();
		SpecGrab();


		// Feedback to Driver Hub
		telemetry.addData("path state", pathState);
		telemetry.addData("x", follower.getPose().getX());
		telemetry.addData("y", follower.getPose().getY());
		telemetry.addData("heading", follower.getPose().getHeading());
		telemetry.addData("specDrive Position", specDrive.getCurrentPosition());
		telemetry.addData("intakeDrive Position", intakeDrive.getCurrentPosition());
		telemetry.addData("Right Sensor Is Pressed", rightTouchSensor.isPressed());
		telemetry.addData("Left Sensor Is Pressed", leftTouchSensor.isPressed());

//		if (intakeDrive.getCurrentPosition()>7) {
//			intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		}

//        telemetry.addData("Intake Position", intakeDrive.getCurrentPosition());
		telemetry.update();
	}

	/**
	 * This method is called once at the init of the OpMode.
	 **/
	@Override
	public void init() {
		pathTimer = new Timer();
		opmodeTimer = new Timer();
		opmodeTimer.resetTimer();
		scoreTimer = new Timer();

		Constants.setConstants(FConstants.class, LConstants.class);
		follower = new Follower(hardwareMap);
		follower.setStartingPose(startPose);
		buildPaths();

		specDrive = hardwareMap.get(DcMotor.class, "specDrive");
		specServo = hardwareMap.get(Servo.class, "specServo");
		rightTouchSensor = hardwareMap.get(TouchSensor.class, "rightTouchSensor");
		leftTouchSensor = hardwareMap.get(TouchSensor.class, "leftTouchSensor");
		intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
		intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		specDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		specDrive.setTargetPosition(150);
		specDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		specDrive.setPower(0.5);
		specServo.setPosition(RConstants.SPECCLAWCLOSED);
		intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		intakeDrive.setPower(0);


		if (opmodeTimer.getElapsedTimeSeconds() > 29.5) {
			specScore = true;
			isSpecScore = true;
			follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
		}

	}

	/**
	 * This method is called continuously after Init while waiting for "play".
	 **/
	@Override
	public void init_loop() {
	}

	/**
	 * This method is called once at the start of the OpMode.
	 * It runs all the setup actions, including building paths and starting the path system
	 **/
	@Override
	public void start() {
		opmodeTimer.resetTimer();
		setPathState(0);
		intakeDrive.setTargetPosition(0);
	}


	/**
	 * We do not use this because everything should automatically disable
	 **/
	@Override
	public void stop() {
	}
}