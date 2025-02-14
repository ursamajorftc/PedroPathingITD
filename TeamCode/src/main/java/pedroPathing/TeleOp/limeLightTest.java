package pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Limelight", group = "Linear OpMode")
public class limeLightTest extends LinearOpMode {
    private FtcDashboard ftcDashboard;
    private Limelight3A limelight;
    private Servo testServo;


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        limelight.setPollRateHz(30);

        testServo = hardwareMap.get(Servo.class, "testServo");
        testServo.setPosition(0);

        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                double angle = pythonOutputs[5];
                double servoPosition = Math.min(1, Math.max(0, Math.abs(angle) / 180));
                testServo.setPosition(servoPosition);

                telemetry.addData("Raw Angle", angle);
                telemetry.addData("Servo Position", servoPosition);
            } else {
                telemetry.addLine("Invalid Python Output");
            }
        }
    }
}
