/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package pedroPathing.TeleOp;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.Actions.IntakeController;
import pedroPathing.pid.PIDController;

@Config

@TeleOp(name = "mainTeleOp", group = "Linear OpMode")

public class mainTeleOpReal extends LinearOpMode {
	// Declare OpMode members.
	private FtcDashboard dashboard;
	private ElapsedTime runtime = new ElapsedTime();
	private CRServo intakeCRSLeft = null;
	private CRServo intakeCRSRight = null;
	private Servo intakeServoLeft = null;
	private Servo intakeServoRight = null;
	private Servo lockServo = null;
	private DcMotor intakeDrive = null;
	private DcMotor specDrive = null;
	private Servo specServo = null;
	private Servo clawServo = null;
	private Servo wristServo = null;
	private Servo armServo = null;
	private DcMotor outmoto1 = null;
	private DcMotor outmoto2 = null;
	private static final int MIN_POSITION = 0;
	private static final int MAX_POSITION = 880;

	//wrist positions
	public double wristPositionDown = 0;
	public double wristPositionStraight = 0.62;
	public double wristPositionOut = 1;

	//arm positions
	public double armPositionDeposit = 0.425;
	public double armPositionHover = 0.815;
	public double armPositionGrab = 0.95;

	//claw positions
	public double clawPositionOpen = 0.26;
	public double clawPositionClosed = 0.48;
	private boolean previousDpadDownState = false;
	private boolean previousDpadUpState = false;
	private boolean PreviousDpadLeftState = false;
	private boolean previousAState = false;
	private boolean previousIntakeState = false;

	boolean sampleDistanceTriggered = false;
	int state = 0; // Persistent state variable
	long startTime = 0; // Persistent timer variable
	NormalizedColorSensor colorSensor;
	NormalizedColorSensor sampleDistance;
	View relativeLayout;
	final float[] hsvValues = new float[3];
	public double intakeServoPosition = 0;
	int highBasket = 1150;
	int lowBasket = 530;
	int downPosition = 0;

	public static double kp = 0.01;
	public static double ki = 0.0;
	public static double kd = 0.0;
	private Follower follower;
	private final Pose startPose = new Pose(0,0,0);


	PIDController pid = new PIDController(kp, kd, ki);

	@Override
	public void runOpMode() throws InterruptedException {

		Constants.setConstants(FConstants.class,LConstants.class);
		follower = new Follower(hardwareMap);
		follower.setStartingPose(startPose);

		dashboard = FtcDashboard.getInstance();
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
		relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

		intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
		intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
		intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
		intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
		lockServo = hardwareMap.get(Servo.class, "lockServo");



		intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
		intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		clawServo = hardwareMap.get(Servo.class, "clawServo");
		wristServo = hardwareMap.get(Servo.class, "wristServo");
		armServo = hardwareMap.get(Servo.class, "armServo");

		outmoto1 = hardwareMap.get(DcMotorEx.class, "outmoto1");
		outmoto2 = hardwareMap.get(DcMotorEx.class, "outmoto2");

		outmoto1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		outmoto2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		outmoto1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		outmoto1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		outmoto2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		specDrive = hardwareMap.get(DcMotor.class, "specDrive");
		specServo = hardwareMap.get(Servo.class,"specServo");

		specDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		specDrive.setTargetPosition(0);
		specDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);





		VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
		colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
		sampleDistance = hardwareMap.get(NormalizedColorSensor.class, "sampleDistance");

		armServo.setPosition(0.475);
		wristServo.setPosition(wristPositionDown);


		// Wait for the game to start (driver presses START)
		waitForStart();

		IntakeController intakeController = new IntakeController(gamepad1, intakeDrive);
		runtime.reset();

		armServo.setPosition(0.475);
		wristServo.setPosition(wristPositionDown);

		follower.startTeleopDrive();


		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {
			boolean sampleDistanceTriggered = false;
			long startTime = 0;
			int state = 0;

			while (opModeIsActive()) {
				follower.setTeleOpMovementVectors(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x * 0.65, true);
				follower.update();

//				drive.setDrivePowers(new PoseVelocity2d(
//						new Vector2d(
//								-gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y) * 1,
//								-gamepad2.left_stick_x * Math.abs(gamepad2.left_stick_x) * 1
//						),
//						(-gamepad2.right_stick_x * Math.abs(gamepad2.right_stick_x) * 0.85)
//				));
				NormalizedRGBA colors = colorSensor.getNormalizedColors();
				NormalizedRGBA colors1 = sampleDistance.getNormalizedColors();

				intakeController.run();

				// Update the hsvValues array by passing it to Color.colorToHSV()
				Color.colorToHSV(colors.toColor(), hsvValues);
				double color = hsvValues[0];


				// Setup a variable for each drive wheel to save power level for telemetry

				if (gamepad2.a){

					specServo.setPosition(0.55);
					sleep(200);
					specDrive.setTargetPosition(420);
					specDrive.setPower(1);
				}
				if (gamepad2.b) {
					specScore = true;
				}
				SpecScore();

				if (gamepad1.x) {
					intakeCRSLeft.setPower(1);
					intakeCRSRight.setPower(-1);
				}
				if (gamepad1.b) {
					intakeCRSLeft.setPower(0);
					intakeCRSRight.setPower(0);
				}
				if ((gamepad1.right_bumper) || (color > 15 && color < 60)) {
					// intakeServoPosition += 0.02;
					intakeServoLeft.setPosition(0.32);
					intakeServoRight.setPosition(0.695);
					intakeCRSLeft.setPower(-0.15);
					intakeCRSRight.setPower(0.15);
				}
				if (intakeDrive.getCurrentPosition() < 50 && previousIntakeState) {
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
					pid.setTargetPosition(highBasket);
				}
				if (gamepad1.dpad_left && PreviousDpadLeftState) {
					pid.setTargetPosition(lowBasket);
				}

				updateArmTransfer();
				updateArmRetracty();
				peckArm();


				if (gamepad1.a) {
					wristServo.setPosition(wristPositionOut);
					clawServo.setPosition(clawPositionOpen);
				}

				if (gamepad2.y) {
					intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				}

				//intakeServoRight.setPosition(intakeServoPosition);
				previousDpadDownState = gamepad1.dpad_down;
				previousDpadUpState = gamepad1.dpad_up;
				PreviousDpadLeftState = gamepad1.dpad_left;


				if ((intakeDrive.getCurrentPosition() > 145) && outmoto1.getCurrentPosition() < 100) {
					previousIntakeState = true;
					if (outmoto1.getCurrentPosition() < 15) {
						armServo.setPosition(armPositionHover);
						clawServo.setPosition(clawPositionOpen);
					}
				}

				double power = pid.getPower(outmoto1.getCurrentPosition());
				if (outmoto1.getCurrentPosition()> pid.getTargetPosition()) {
					outmoto1.setPower(-0.2);
					outmoto2.setPower(0.2);
				} else {
					outmoto1.setPower(power);
					outmoto2.setPower(-power);
				}

				TelemetryPacket packet = new TelemetryPacket();

				packet.put("Status", "Run Time: " + runtime.toString());
				packet.put("IntakeServoPosition", intakeServoPosition);
				packet.put("IntakePosition", intakeDrive.getCurrentPosition());
				packet.put("HSV Value", color);
				packet.put("Distance", ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM));
				packet.put("Deposit Slides", outmoto1.getCurrentPosition());
				double batteryVoltage = voltageSensor.getVoltage();
				packet.put("Battery Voltage", batteryVoltage);
				packet.put("Deposit position", outmoto1.getCurrentPosition());
				packet.put("Target Position", highBasket );
				dashboard.sendTelemetryPacket(packet);

				telemetry.addData("Battery Voltage", batteryVoltage);
				telemetry.update();
				telemetry.addData("Status", "Run Time: " + runtime.toString());
				telemetry.addData("IntakeServoPosition", intakeServoPosition);
				telemetry.addData("IntakePosition", intakeDrive.getCurrentPosition());
				telemetry.addData("hsv Value:", color);
				telemetry.addData("Distance", ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM));
				telemetry.addData("Deposit Slides", outmoto1.getCurrentPosition());
				telemetry.addData("Deposit Target Position", pid.getTargetPosition());
				telemetry.addData("X", follower.getPose().getX());
				telemetry.addData("Y", follower.getPose().getY());
				telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
				telemetry.addData("specDrive position", specDrive.getCurrentPosition());
				telemetry.update();
			}
		}
	}

	// to make sure that the thread doesn't hinder other operations
	int peckState = 0;
	long peckTime = 0;

	public boolean specScore = false;
	public boolean specPickUp = false;
	public long specTimer = 0;

	public void speckPickUp() {
	}
	public void SpecScore() {
		if (specScore) {
			specDrive.setTargetPosition(0);
			specDrive.setPower(0.4);
		}

		if ((specDrive.getTargetPosition() == 0) && specDrive.getCurrentPosition() < 250) {
			specServo.setPosition(0.3);
			specScore = false;
		}
	}

	public void peckArm() {
		if (gamepad2.right_bumper) {
			peckState = 1;
			peckTime = System.currentTimeMillis();
		}

		switch (peckState) {
			case 1:
				armServo.setPosition(armPositionGrab);
				wristServo.setPosition(wristPositionDown);
				peckState = 2;
				break;

			case 2:
				if (System.currentTimeMillis() - peckTime > 100) {
					armServo.setPosition(armPositionHover);
					wristServo.setPosition(wristPositionDown);
					peckState = 0;
				}
		}
	}

	public void updateArmTransfer() {
		if ((!sampleDistanceTriggered && ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) < 15)) {
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

						clawServo.setPosition(clawPositionClosed);
						state = 3;
						startTime = System.currentTimeMillis(); // Reset timer
					}
					break;

				case 3:
					if (elapsedTime >= 200) {

						armServo.setPosition(0.475);
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


	public void updateArmRetracty() {
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
					pid.setTargetPosition(downPosition);
					if (outmoto1.getCurrentPosition()>=20) {
						outmoto1.setPower(-0.2);
						outmoto2.setPower(0.2);
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
}
