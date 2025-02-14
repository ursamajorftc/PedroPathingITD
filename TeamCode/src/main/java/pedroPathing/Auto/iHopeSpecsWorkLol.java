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
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotor intakeDrive = null;
    private DcMotor specDrive = null;
    private Servo specServo = null;

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;


    private final Pose startPose = new Pose(8.401, 68, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(28.5, 73, Math.toRadians(185));
    private final Pose startPush1 = new Pose(38, 47, Math.toRadians(180));
    private final Pose endPush1 = new Pose(22, 42, Math.toRadians(185));
    private final Pose endPush2 = new Pose(22, 32, Math.toRadians(185));
    private final Pose startPush3 = new Pose(47, 29, Math.toRadians(185));
    private final Pose endPush3 = new Pose(22, 29, Math.toRadians(185));
    private final Pose scorePose2 = new Pose(26.4, 72, Math.toRadians(185));
    private final Pose grabPose = new Pose(9, 42, Math.toRadians(185));
    private final Pose scorePose3 = new Pose(26.4, 71, Math.toRadians(185));
    private final Pose scorePose4 = new Pose(26.4, 70, Math.toRadians(185));
    private final Pose scorePose5 = new Pose(26.4, 69, Math.toRadians(185));

    /**
     * Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve.
     */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

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
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose1)));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading());

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
                .setConstantHeadingInterpolation(Math.toRadians(185))
                // push sample 1
                .addPath(
                        new BezierCurve(
                                new Point(startPush1),
                                new Point(55, 36, Point.CARTESIAN),
                                new Point(endPush1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(185))
                //push 2
                .addPath(
                        new BezierCurve(
                                new Point(endPush1),
                                new Point(65, 39.5, Point.CARTESIAN),
                                new Point(endPush2)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(185))
                // get to point before pushing 3
                .addPath(
                        new BezierCurve(
                                new Point(endPush2),
                                new Point(52, 38, Point.CARTESIAN),
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
                ).setConstantHeadingInterpolation(Math.toRadians(185))
                .addPath(
                        new BezierLine(
                                new Point(endPush3),
                                new Point(grabPose)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(185))

                .build();
        score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(scorePose2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(185))
                .build();
        grab3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose2),
                                new Point(grabPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(185))
                .build();
        score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(scorePose3)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(185))
                .build();
        grab4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose3),
                                new Point(grabPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(185))
                .build();
        score4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(scorePose4)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(185))
                .build();
        grab5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose4),
                                new Point(grabPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(185))
                .build();
        score5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(scorePose5)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(185))
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

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose1's position */
                if (!follower.isBusy() && !isSpecScore) {
//
                    /* Score Preload */
                    specScore = true;
                    isSpecScore = true;

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                }

                if ((specDrive.getCurrentPosition() < 200) && !follower.isBusy() && isSpecScore) {
                    follower.followPath(pushSamples, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the endPush1's position */
                if (!follower.isBusy() && !isSpecGrabbed) {

                    specGrabbed = false;
                    isSpecGrabbed = true;
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                }
//
//                if (specDrive.getCurrentPosition() > 100) {
//                    follower.followPath(score2, true);
//                    setPathState(3);
//                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose1's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grab3, true);
//                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose1's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(score3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    /* Action */

                    /* Path */
                    follower.followPath(grab4, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    /* Action */

                    /* Path */
                    follower.followPath(score4, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    /* Action */

                    /* Path */
                    follower.followPath(grab5, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    /* Action */

                    /* Path */
                    follower.followPath(score5, true);
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose1's position */
                if (!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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

        if (specScore &&(specDrive.getTargetPosition() == 0) && specDrive.getCurrentPosition() < 280) {
            specServo.setPosition(0.3);
            specScore = false;
        }
    }

    public void SpecGrab() {
        if (!specGrabbed) {
            switch (specState){
                case 1:
                    specServo.setPosition(RConstants.SPECCLAWCLOSED);
                    specState = 2;
                    specTimeStart = System.currentTimeMillis();
                    break;
                case 2:
                    if ((System.currentTimeMillis() - specTimeStart) > 1200){
                        specDrive.setTargetPosition(RConstants.SPECARMUP);
                        specDrive.setPower(1);
                        specState = 1;
                        specGrabbed = true;
                    }
            }
        }



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
        telemetry.addData("score spec boolean", specScore);
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        specDrive = hardwareMap.get(DcMotor.class, "specDrive");
        specServo = hardwareMap.get(Servo.class, "specServo");
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        specDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeDrive.setTargetPosition(0);
        specDrive.setTargetPosition(200);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        specDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(0.5);
        specDrive.setPower(0.5);
        specServo.setPosition(RConstants.SPECCLAWCLOSED);

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
        intakeDrive.setTargetPosition(10);


    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}