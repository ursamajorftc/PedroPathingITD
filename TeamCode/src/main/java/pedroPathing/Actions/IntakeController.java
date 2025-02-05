package pedroPathing.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class IntakeController {
	private static final int MIN_POSITION = 0;
	private static final int MAX_POSITION = 880;
	int intakeTargetPosition = 0;

	Gamepad gamepad1;
	DcMotor intakeDrive;

	public IntakeController(Gamepad gamepad1, DcMotor intakeDrive) {
		this.gamepad1 = gamepad1;
		this.intakeDrive = intakeDrive;
	}

	public void run() {
		double joystickInput = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y) ;
		int intakePosition = intakeDrive.getCurrentPosition();
		

		intakeTargetPosition = intakeTargetPosition + (int)(joystickInput * 80);

		if (intakeTargetPosition < MIN_POSITION) {
			intakeTargetPosition = MIN_POSITION;
		} else if (intakeTargetPosition > MAX_POSITION) {
			intakeTargetPosition = MAX_POSITION;
		}
		if (gamepad1.y){
			intakeTargetPosition = 0;
		}
		intakeDrive.setTargetPosition(intakeTargetPosition);
		intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		intakeDrive.setPower(1);


		intakeDrive.setTargetPosition(intakeTargetPosition);
	}


}
