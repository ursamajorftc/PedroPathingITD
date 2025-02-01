//region Imports
package pedroPathing.TeleOp;
import android.view.View;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import pedroPathing.constants.*;
import pedroPathing.pid.PIDController;
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
	private Servo intakeServoRight = null;
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
	public void start() {follower.startTeleopDrive();}

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