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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.constants.RConstants;
import pedroPathing.pid.PIDController;


// This is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking in the submersible zone.

@Autonomous(name = "4 Samples", group = "Examples")
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
    private PIDController backPid = new PIDController(0.04, 0, 0.009);
    private boolean previousAState = false;
    private double power;
    private boolean scored = false;
    private boolean safe = false;
    private long stateStartTime = 0;
    private boolean downState = false;
    private boolean firstTime = true;
    private boolean previousIntakeState = false;
    private boolean intakeComplete = false;
    long intakeStartTime = 0;
    private boolean pickedUp = false;
    boolean sampleDistanceTriggered = false;
    boolean intakeJerk = false;
    private boolean transferComplete = false;
    private RConstants.DepositState depositState = RConstants.DepositState.DOWN;


    long startTime = 0; // Persistent timer variable


    NormalizedColorSensor sampleDistance;



    private int state = 0;


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
    private final Pose scorePose = new Pose(14.959, 127.865, Math.toRadians(315));

    /**
     * Lowest (First) Sample from the Spike Mark
     */
    private final Pose pickup1Pose = new Pose(21, 123.2, Math.toRadians(0));

    /**
     * Middle (Second) Sample from the Spike Mark
     */
    private final Pose pickup2Pose = new Pose(21, 130.2, Math.toRadians(0));

    /**
     * Highest (Third) Sample from the Spike Mark
     */
    private final Pose pickup3Pose = new Pose(21, 129, Math.toRadians(30));

    /**
     * Park Pose for our robot, after we do all of the scoring.
     */
    private final Pose parkPose = new Pose(46, 114, Math.toRadians(270));

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

        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setTargetPosition(0);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(0.5);

        // Deposit
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        outmoto1 = hardwareMap.get(DcMotorEx.class, "outmoto1");
        outmoto2 = hardwareMap.get(DcMotorEx.class, "outmoto2");
        sampleDistance = hardwareMap.get(NormalizedColorSensor.class, "sampleDistance");

        outmoto1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outmoto1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armServo.setPosition(RConstants.ARMPOSITIONDEPOSIT);
        wristServo.setPosition(RConstants.WRISTPOSITIONSTRAIGHT);
        clawServo.setPosition(RConstants.CLAWPOSITIONCLOSED);

        intakeServoLeft.setPosition(0.32);
        intakeServoRight.setPosition(0.695);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        buildPaths();


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

                follower.followPath(scorePreload, true);
                backPid.setTargetPosition(1153);


                setPathState(1);


//				backPid.setTargetPosition(1150);

                break;
            case 1:
                if (!follower.isBusy() && !scored && (outmoto1.getCurrentPosition() > 1140)) {
                    armServo.setPosition(RConstants.ARMPOSITIONDEPOSIT);
                    wristServo.setPosition(RConstants.WRISTPOSITIONOUT);
                    clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
                    downState = true;
                    scored = true;
                    if (firstTime) {
                        stateStartTime = System.currentTimeMillis();
                        firstTime = false;
                    }
                }
                updateArmRetracty();


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
                    safe = false;
                    scored = false;
                    firstTime = true;


                    setPathState(2);
                }
                break;
            case 2:


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy() && !pickedUp) {

                    intakeState = RConstants.IntakeState.INTAKE0;
                    pickedUp = true;

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                }
                intakeMovement(790); //picks up the sample

                if (intakeComplete) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                    transferComplete = false;

                }




                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    if (!transferComplete) {
                        updateArmTransfer();
                    }
                    /* Score Sample *///	scoreSample();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2, true);
//                    setPathState(4);
                }

                if (transferComplete && depositState.equals(RConstants.DepositState.DOWN)) {
                    backPid.setTargetPosition(1160);
                    depositState = RConstants.DepositState.UP;
                }

                if (!scored && (outmoto1.getCurrentPosition() > 1150)) {
                    armServo.setPosition(RConstants.ARMPOSITIONDEPOSIT);
                    wristServo.setPosition(RConstants.WRISTPOSITIONOUT);
                    clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
                    downState = true;
                    scored = true;
                    if (firstTime) {
                        stateStartTime = System.currentTimeMillis();
                        firstTime = false;
                    }
                }
                updateArmRetracty();


                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"`
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (safe || (jerkCount>2)) {
                    safe = false;
                    /* Score Preload */
//					scoreSample();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                    intakeComplete = false;
                    pickedUp = false;
                    scored = false;
                    firstTime = true;
                    jerkCount = 0;
                    intakeJerk = false;
                    intakeCRSLeft.setPower(1);
                    intakeCRSRight.setPower(-1);
                    intakeDrive.setTargetPosition(0);






                }


                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy() && !pickedUp) {

                    intakeState = RConstants.IntakeState.INTAKE0;
                    pickedUp = true;

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                }
                intakeMovement(720); //picks up the sample

                if (intakeComplete) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                    transferComplete = false;
                    depositState = RConstants.DepositState.DOWN;

                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    if (!transferComplete) {
                        telemetry.addData("transfer", "");
                        updateArmTransfer();
                    }
                    /* Score Sample *///	scoreSample();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2, true);
//                    setPathState(4);
                }

                if (transferComplete && depositState.equals(RConstants.DepositState.DOWN)) {
                    backPid.setTargetPosition(1160);
                    depositState = RConstants.DepositState.UP;

                }

                if (!scored && (outmoto1.getCurrentPosition() > 1150)) {
                    armServo.setPosition(RConstants.ARMPOSITIONDEPOSIT);
                    wristServo.setPosition(RConstants.WRISTPOSITIONOUT);
                    clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
                    telemetry.addData("open claw", 1);
                    telemetry.update();
                    downState = true;
                    scored = true;
                    if (firstTime) {
                        stateStartTime = System.currentTimeMillis();
                        firstTime = false;
                    }
                }
                updateArmRetracty();


                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"`
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (safe || (jerkCount>2)) {
                    safe = false;
                    /* Score Preload */
//					scoreSample();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                    intakeComplete = false;
                    pickedUp = false;
                    scored = false;
                    jerkCount = 0;
                    intakeJerk = false;
                    intakeCRSLeft.setPower(1);
                    intakeCRSRight.setPower(-1);
                    intakeDrive.setTargetPosition(0);





                }

                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy() && !pickedUp) {

                    intakeState = RConstants.IntakeState.INTAKE0;
                    pickedUp = true;

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                }
                intakeMovement(670); //picks up the sample

                if (intakeComplete) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                    transferComplete = false;
                    firstTime = true;
                    depositState = RConstants.DepositState.DOWN;

                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    if (!transferComplete) {
                        updateArmTransfer();
                    }
                    /* Score Sample *///	scoreSample();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2, true);
//                    setPathState(4);
                }

                if (transferComplete && depositState.equals(RConstants.DepositState.DOWN)) {
                    backPid.setTargetPosition(1160);
                    depositState = RConstants.DepositState.UP;

                }

                if (!scored && (outmoto1.getCurrentPosition() > 1150)) {
                    armServo.setPosition(RConstants.ARMPOSITIONDEPOSIT);
                    wristServo.setPosition(RConstants.WRISTPOSITIONOUT);
                    clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
                    telemetry.addData("open claw", 1);
                    telemetry.update();
                    downState = true;
                    scored = true;
                    if (firstTime) {
                        stateStartTime = System.currentTimeMillis();
                        firstTime = false;
                    }
                }
                updateArmRetracty();


                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"`
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (safe|| (jerkCount>2)) {
                    safe = false;
                    /* Score Preload */
//					scoreSample();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(park, true);
                    setPathState(8);
                    intakeComplete = false;
                    pickedUp = false;
                    scored = false;
                    jerkCount = 0;
                    intakeJerk = false;

                    intakeCRSLeft.setPower(1);
                    intakeCRSRight.setPower(-1);
                    intakeDrive.setTargetPosition(0);



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


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("outmoto1 position", outmoto1.getCurrentPosition());
        telemetry.addData("intakeDrive position", intakeDrive.getCurrentPosition());
        telemetry.addData("intakeState", intakeState);

        telemetry.addData("downState", downState);
        telemetry.update();

        power = backPid.getPower(outmoto1.getCurrentPosition());
        if (outmoto1.getCurrentPosition() > backPid.getTargetPosition()) {
            outmoto1.setPower(-0.2);
            outmoto2.setPower(0.2);
        } else {
            outmoto1.setPower(power);
            outmoto2.setPower(-power);
        }

        if (intakeDrive.getCurrentPosition() < 50 && previousIntakeState) {
            lockServo.setPosition(0.3);
            intakeCRSLeft.setPower(-0.2);
            intakeCRSRight.setPower(0.2);
            previousIntakeState = false;
        }
        if ((intakeDrive.getCurrentPosition() > 245) && outmoto1.getCurrentPosition() < 100) {
            previousIntakeState = true;
            if (outmoto1.getCurrentPosition() < 15) {
                armServo.setPosition(RConstants.ARMPOSITIONHOVER);
                clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
            }
        }



    }

    private RConstants.RobotState currentState = RConstants.RobotState.IDLE;



    public void updateArmRetracty() {

        // Get the current time in milliseconds
        long currentTime = System.currentTimeMillis();


        switch (currentState) {
            case IDLE:

                if (downState && (currentTime - stateStartTime > 400)) {
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
                    safe = true;
                    downState = false;
                    stateStartTime = 0;

                }
                // All actions complete; stay idle or transition as needed
                break;
        }
    }

    private RConstants.IntakeState intakeState = RConstants.IntakeState.INTAKESTOP;


    public void intakeMovement(int intakeTargetPosition) {
        if ((intakeDrive.getCurrentPosition() > 145) && outmoto1.getCurrentPosition() < 100) {
            previousIntakeState = true;
            if (outmoto1.getCurrentPosition() < 15) {
                armServo.setPosition(RConstants.ARMPOSITIONHOVER);
                clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
            }
        }


        switch (intakeState) {

            case INTAKE0:

                intakeDrive.setTargetPosition(300);
                intakeState = RConstants.IntakeState.INTAKE300;
                intakeDrive.setPower(1);

                break;

            case INTAKE300:
                if (intakeDrive.getCurrentPosition() > 250) {
                    telemetry.addData("yippee", intakeState);
                    telemetry.update();

                    intakeServoLeft.setPosition(0.54);
                    intakeServoRight.setPosition(0.45);
                    intakeCRSLeft.setPower(-1);
                    intakeCRSRight.setPower(1);
                    lockServo.setPosition(0);
                    intakeState = RConstants.IntakeState.INTAKEDOWN;
                    intakeDrive.setPower(0.2);
                }
                break;

            case INTAKEDOWN:
                intakeDrive.setTargetPosition(intakeTargetPosition);
                intakeState = RConstants.IntakeState.INTAKE880;

                break;

            case INTAKE880:
                if (intakeDrive.getCurrentPosition() > (intakeTargetPosition - 10)) {
                    intakeDrive.setTargetPosition(0);
                    intakeDrive.setPower(1);
                    intakeServoLeft.setPosition(0.32);
                    intakeServoRight.setPosition(0.695);
                    intakeCRSLeft.setPower(-0.1);
                    intakeCRSRight.setPower(0.1);
                    intakeState = RConstants.IntakeState.INTAKESTOP;
                    intakeComplete = true;

                    intakeStartTime = System.currentTimeMillis();

                }

                break;


        }
    }

    public int jerkCount = 0;

    public void updateArmTransfer() {
        if (intakeDrive.getCurrentPosition() < 50) {
            lockServo.setPosition(0.3);
            intakeCRSLeft.setPower(-0.25);
            intakeCRSRight.setPower(0.25);
        }

        if ((!sampleDistanceTriggered && (((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) > 20) && !intakeJerk && intakeDrive.getCurrentPosition() <= 0) ) {

            intakeDrive.setTargetPosition(200);
            intakeJerk = true;
            jerkCount += 1;
//            ((System.currentTimeMillis()-intakeStartTime)>2000)


        }



        if (intakeJerk && intakeDrive.getCurrentPosition()>90){
            intakeDrive.setTargetPosition(-20);
            intakeJerk = false;

        }


        if ((!sampleDistanceTriggered && ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) < 20) ) {
            intakeJerk = false;
            sampleDistanceTriggered = true;
            startTime = System.currentTimeMillis();
            state = 1;
            // Start the state machine
        }


        if (sampleDistanceTriggered) {
            long elapsedTime = System.currentTimeMillis() - startTime;

            switch (state) {
                case 1:
                    armServo.setPosition(RConstants.ARMPOSITIONGRAB);
                    clawServo.setPosition(RConstants.CLAWPOSITIONOPEN);
                    wristServo.setPosition(RConstants.WRISTPOSITIONDOWN);
                    state = 2;
                    startTime = System.currentTimeMillis(); // Reset timer
                    break;

                case 2:
                    if (elapsedTime >= 150) {

                        clawServo.setPosition(RConstants.CLAWPOSITIONCLOSED);
                        state = 3;
                        startTime = System.currentTimeMillis(); // Reset timer
                    }
                    break;

                case 3:
                    if (elapsedTime >= 200) {

                        armServo.setPosition(0.475);
                        wristServo.setPosition(0);
                        state = 4;
                        startTime = System.currentTimeMillis(); // Reset timer
                    }
                    break;

                case 4:
                    if (elapsedTime >= 250) {
                        wristServo.setPosition(RConstants.WRISTPOSITIONSTRAIGHT);
                        if (((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) < 17) {
                            state = 1;
                        } else {
                            state = 0;
                            sampleDistanceTriggered = false;
                            transferComplete = true;
                            jerkCount = 0;

                        }


                        // End the state machine
                    }
                    break;
            }
        }
    }



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