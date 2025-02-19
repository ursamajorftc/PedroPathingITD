package pedroPathing.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Config

@TeleOp(name = "intake test", group = "Linear OpMode")
public class testIntake extends LinearOpMode {
    DcMotor intakeDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                intakeDrive.setTargetPosition(200);
                intakeDrive.setPower(0.2);
            } else if (gamepad1.y) {
                intakeDrive.setTargetPosition(0);
                intakeDrive.setPower(0.2);
            }
        }
    }
}
