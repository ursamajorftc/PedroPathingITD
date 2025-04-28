package pedroPathing.TeleOp;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.constants.RConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.util.Constants;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

import pedroPathing.pid.PIDController;

@Config
@TeleOp(name = "Main TeleOp", group = "Linear OpMode")

public class mainTeleOpReal extends LinearOpMode {
    // region Initializations
    private Follower follower;
    private FtcDashboard dashboard;
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo intakeCRSLeft;
    private CRServo intakeCRSRight;
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private Servo lockServo;
    private DcMotor intakeDrive;
    private DcMotor specDrive;
    private Servo specServo;
    private Servo clawServo;
    private Servo wristServo;
    private Servo armServo;
    private DcMotor outMotor1;
    private DcMotor outMotor2;
    NormalizedColorSensor colorSensor;
    NormalizedColorSensor sampleDistance;
    TouchSensor rightTouchSensor;
    TouchSensor leftTouchSensor;
    View relativeLayout;
    PIDController verticalSlidePid = new PIDController(0.01, 0, 0);
    //endregion

    //region Variables
    //wrist positions
    public double wristPositionDown = 0;
    public double wristPositionStraight = 0.62;
    public double wristPositionOut = 1;

    //arm positions
    public double armPositionDeposit = 0.475;
    public double armPositionHover = 0.815;
    public double armPositionGrab = 0.95;

    //claw positions
    public double clawPositionOpen = 0.26;
    public double clawPositionClosed = 0.48;

    //vertical slide positions
    int highBasket = 1150;
    int lowBasket = 530;
    int downPosition = 0;

    //state machines
    private boolean previousBumperState = false;
    private boolean previousDpadUpState = false;
    private boolean PreviousDpadLeftState = false;
    private boolean previousAState = false;
    private boolean previousIntakeState = false;
    boolean sampleDistanceTriggered = false;
    int state = 0; // Persistent state variable
    long startTime = 0; // Persistent timer variable

    final float[] hsvValues = new float[3];
    public double intakeServoPosition = 0;

    private final Pose robotPose = new Pose(29, 67, Math.toRadians(180));
    private double scorePosition = 65;
    private boolean intakeRetract = false;
    private boolean specMode = false;
    //endregion

    @Override
    public void runOpMode() throws InterruptedException {

        //region Declarations
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(robotPose);

        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        buildPaths();

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        lockServo = hardwareMap.get(Servo.class, "lockServo");

        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");

        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        outMotor1 = hardwareMap.get(DcMotorEx.class, "outMotor1");
        outMotor2 = hardwareMap.get(DcMotorEx.class, "outMotor2");

        outMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        MotorGroup backSlides = new MotorGroup(outMotor1, outMotor2);

        specDrive = hardwareMap.get(DcMotor.class, "specDrive");
        specServo = hardwareMap.get(Servo.class, "specServo");

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        sampleDistance = hardwareMap.get(NormalizedColorSensor.class, "sampleDistance");

        rightTouchSensor = hardwareMap.get(TouchSensor.class, "rightTouchSensor");
        leftTouchSensor = hardwareMap.get(TouchSensor.class, "leftTouchSensor");

        armServo.setPosition(0.475);
        wristServo.setPosition(wristPositionDown);

        intakeDrive.setPower(-0.2);
        specDrive.setPower(-0.4);
        specServo.setPosition(RConstants.SPECCLAWOPEN);
        //endregion

        // Wait for the game to start (driver presses START)
        waitForStart();

        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        specDrive.setTargetPosition(0);
        specDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        armServo.setPosition(0.475);
        wristServo.setPosition(wristPositionDown);

        follower.startTeleopDrive();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        scoreTimer = new Timer();


        while (opModeIsActive()) {
            resetCache(allHubs);

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);
            double color = hsvValues[0];

            if (gamepad2.a) {
                specServo.setPosition(0.55);
                sleep(400);
                specDrive.setTargetPosition(490);
                specDrive.setPower(1);
            }
            if (gamepad2.b) {
                specScore = true;
            }
            SpecScore();

            if (gamepad1.x) {
                intakeCRSLeft.setPower(1);
                intakeCRSRight.setPower(-1);
            } else {
                intakeCRSLeft.setPower(-1);
                intakeCRSRight.setPower(1);
            }

            if (gamepad1.y){
                intakeRetract = true;
                if (intakeRetract) {intakeDrive.setPower(-0.6);}
            } else {
                intakeRetract = false;
            }

            if (gamepad1.b) {
                intakeCRSLeft.setPower(0);
                intakeCRSRight.setPower(0);
            }
            if ((gamepad1.right_bumper) || (color > 15 && color < 60)) {
                intakeServoLeft.setPosition(0.32);
                intakeServoRight.setPosition(0.695);
                intakeCRSLeft.setPower(-0.15);
                intakeCRSRight.setPower(0.15);
            }
            if (intakeDrive.getCurrentPosition() < 50 && previousIntakeState && !specMode) {
                lockServo.setPosition(0.3);
                intakeCRSLeft.setPower(-0.2);
                intakeCRSRight.setPower(0.2);
                previousIntakeState = false;
            }
            if ((gamepad1.right_trigger > 0.25)) {
                intakeServoLeft.setPosition(0.54);
                intakeServoRight.setPosition(0.45);
                lockServo.setPosition(0);
                intakeCRSLeft.setPower(-1);
                intakeCRSRight.setPower(1);
            }
            if (gamepad1.dpad_up && previousDpadUpState) {
                verticalSlidePid.setTargetPosition(highBasket);
            }
            if (gamepad1.dpad_left && PreviousDpadLeftState) {
                verticalSlidePid.setTargetPosition(lowBasket);
            }

            updateArmTransfer();
            updateArmRetract();

            if (gamepad1.a) {
                wristServo.setPosition(wristPositionOut);
                clawServo.setPosition(clawPositionOpen);
            }

            if (gamepad2.y) {
                intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //intakeServoRight.setPosition(intakeServoPosition);
            previousDpadUpState = gamepad1.dpad_up;
            PreviousDpadLeftState = gamepad1.dpad_left;


            if ((intakeDrive.getCurrentPosition() > 145) && outMotor1.getCurrentPosition() < 100) {
                previousIntakeState = true;
                if (outMotor1.getCurrentPosition() < 15) {
                    armServo.setPosition(armPositionHover);
                    clawServo.setPosition(clawPositionOpen);
                }
            }

            double power = verticalSlidePid.getPower(outMotor1.getCurrentPosition());
            if (outMotor1.getCurrentPosition() > verticalSlidePid.getTargetPosition() + 20) {
                outMotor1.setPower(-0.4);
                outMotor2.setPower(0.4);
            } else if (outMotor1.getCurrentPosition() < verticalSlidePid.getTargetPosition()+20 && outMotor1.getCurrentPosition() >= verticalSlidePid.getTargetPosition()){
                outMotor1.setPower(0);
                outMotor2.setPower(0);
            } else {
                outMotor1.setPower(power);
                outMotor2.setPower(-power);
            }

            if (gamepad2.left_bumper && !previousBumperState) {
                lockServo.setPosition(0);
                specMode = !specMode;
            }

            previousBumperState = gamepad2.left_bumper;

            double joystickInput = (gamepad1.left_stick_y)*-0.8;
            if (!intakeRetract) {
                if (-20 < intakeDrive.getCurrentPosition() && intakeDrive.getCurrentPosition() <= 850) {
                    intakeDrive.setPower(joystickInput);
                } else if (intakeDrive.getCurrentPosition() > 845) {
                    intakeDrive.setPower(-0.6);
                } else {
                    intakeDrive.setPower(0.6);
                }
            }

            if (gamepad2.x) {
                follower.setPose(grabPose);
                setPathState(1);
                isSpecGrabbed = false;
            }

            if (rightTouchSensor.isPressed() && leftTouchSensor.isPressed()) {
                follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(180)));
            }

            while (gamepad2.right_trigger > 0.1) {
                //follow path
                specScoreUpdate();
                telemetry.addData("it is pressed", true);
                telemetry.update();
            }

            if (gamepad2.right_trigger < 0.1) {
                follower.setTeleOpMovementVectors(-gamepad2.left_stick_y, -gamepad2.left_stick_x, (-gamepad2.right_stick_x * 0.65), false);
                follower.update();
                telemetry.addData("it is pressed", false);
                telemetry.update();

            }

            if (specMode) {
                telemetry.addLine("<font color='blue'>Special Mode Active</font>");
            } else {
                telemetry.addLine("<font color='white'>Normal Mode</font>");
            }

            //region Telemetry
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("Status", "Run Time: " + runtime.toString());
            packet.put("IntakeServoPosition", intakeServoPosition);
            packet.put("IntakePosition", intakeDrive.getCurrentPosition());
            packet.put("HSV Value", color);
            packet.put("Deposit Slides", outMotor1.getCurrentPosition());
            double batteryVoltage = voltageSensor.getVoltage();
            packet.put("Battery Voltage", batteryVoltage);
            packet.put("Target Position", highBasket);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Battery Voltage", batteryVoltage);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("IntakePosition", intakeDrive.getCurrentPosition());
            telemetry.addData("HSV Value:", color);
            telemetry.addData("Deposit Slides", outMotor1.getCurrentPosition());
            telemetry.addData("Deposit Target Position", verticalSlidePid.getTargetPosition());
            telemetry.addData("specDrive position", specDrive.getCurrentPosition());
            telemetry.addData("Spec Mode", specMode);
            telemetry.update();
            //endregion
        }
    }
    public boolean specScore = false;

    public void SpecScore() {
        if (specScore) {
            specDrive.setTargetPosition(0);
            specDrive.setPower(0.8);
        }

        if ((specDrive.getTargetPosition() == 0) && specDrive.getCurrentPosition() < 320) {
            specServo.setPosition(0.3);
            specScore = false;
        }
    }
    public boolean specGrabbed = false;

    public void updateArmTransfer() {
        if ((!sampleDistanceTriggered && ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) < 12)) {
            sampleDistanceTriggered = true;
            startTime = System.currentTimeMillis();
            state = 1; // Start the state machine
        }

        if (sampleDistanceTriggered) {
            long elapsedTime = System.currentTimeMillis() - startTime;

            switch (state) {
                case 1:
                    armServo.setPosition(armPositionGrab);
                    clawServo.setPosition(clawPositionOpen);
                    wristServo.setPosition(wristPositionDown);
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

                        armServo.setPosition(armPositionDeposit);
                        state = 4;
                        startTime = System.currentTimeMillis(); // Reset timer
                    }
                    break;

                case 4:
                    if (elapsedTime >= 750) {
                        wristServo.setPosition(wristPositionStraight);
                        state = 0;
                        sampleDistanceTriggered = false;

                        // End the state machine
                    }
                    break;
            }
        }
    }

    enum RobotState {
        IDLE,
        CLOSE_CLAW,
        MOVE_ARM,
        MOVE_WRIST,
        OPEN_CLAW,
        COMPLETE
    }

    private RobotState currentState = RobotState.IDLE;
    private long stateStartTime = 0;

    public void updateArmRetract() {
        // Get the current time in milliseconds
        long currentTime = System.currentTimeMillis();

        switch (currentState) {
            case IDLE:
                if (gamepad1.dpad_down && !previousAState) {
                    // Transition to CLOSE_CLAW state
                    clawServo.setPosition(clawPositionClosed);
                    stateStartTime = currentTime; // Record the time
                    currentState = RobotState.CLOSE_CLAW;
                }
                break;

            case CLOSE_CLAW:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    armServo.setPosition(0.475);

                    currentState = RobotState.MOVE_ARM;
                }
                break;

            case MOVE_ARM:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    wristServo.setPosition(wristPositionDown);
                    stateStartTime = currentTime;
                    currentState = RobotState.MOVE_WRIST;
                }
                break;

            case MOVE_WRIST:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    clawServo.setPosition(clawPositionOpen);
                    stateStartTime = currentTime;
                    currentState = RobotState.OPEN_CLAW;
                }
                break;

            case OPEN_CLAW:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    verticalSlidePid.setTargetPosition(downPosition);
                    if (outMotor1.getCurrentPosition() >= 20) {
                        outMotor1.setPower(-0.2);
                        outMotor2.setPower(0.2);
                    }
                    telemetry.addData("yippee", gamepad1.a);
                    stateStartTime = currentTime;
                    currentState = RobotState.COMPLETE;
                }
                break;

            case COMPLETE:
                if (currentTime - stateStartTime > 200) {
                    currentState = RobotState.IDLE;
                }
                // All actions complete; stay idle or transition as needed
                break;
        }
    }

    private int pathState;
    private Timer pathTimer, opmodeTimer, scoreTimer;


    private final Pose grabPose = new Pose(7.7, 36, Math.toRadians(180));
    private final Pose scorePose = new Pose(29, scorePosition, Math.toRadians(180));
    private final Pose scoreControl = new Pose(25.5, 70);

    private PathChain score, grab;

    public void buildPaths() {
        score = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(grabPose),
                                new Point(scoreControl),
                                new Point(scorePose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grab = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(scoreControl),
                                new Point(grabPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public boolean isSpecScore = false;
    public boolean isSpecGrabbed = false;
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void specScoreUpdate() {
        switch (pathState){
            case 1:
                if ((!follower.isBusy() || (follower.getPose().getX() + 0.1 <= grabPose.getX())) && !isSpecGrabbed) {
                    telemetry.addData("it is running", true);
                    telemetry.update();

                    specGrabbed = false;
                    isSpecGrabbed = true;
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                }

                if ((specServo.getPosition() == RConstants.SPECCLAWCLOSED) && (specDrive.getCurrentPosition() > 290)) {
                    follower.followPath(score,0.9, true);
                    scoreTimer.resetTimer();
                    setPathState(2);
                    isSpecScore = false;
                }
                break;

            case 2:
                if ((rightTouchSensor.isPressed() && leftTouchSensor.isPressed()) && !isSpecScore) {
                    /* Score Preload */
                    specScore = true;
                    isSpecScore = true;
                    follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));

                    telemetry.addData("specScore", specScore);
                } else if ((rightTouchSensor.isPressed() ^ leftTouchSensor.isPressed()) && scoreTimer.getElapsedTimeSeconds() >= 2.5) {
                    specScore = true;
                    isSpecScore = true;
                    follower.setPose(new Pose(28.5, follower.getPose().getY(), Math.toRadians(175)));
                }

                if ((specDrive.getCurrentPosition() < 290) && isSpecScore) {
                    follower.followPath(grab, true);
                    setPathState(1);
                    isSpecGrabbed = false;
                    scorePosition -= 0.5;
                }
                break;
        }
    }
    public void resetCache(List<LynxModule> allHubs){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}