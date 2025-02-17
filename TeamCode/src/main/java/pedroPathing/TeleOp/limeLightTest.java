package pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@Config
@TeleOp(name = "Limelight", group = "Linear OpMode")
public class limeLightTest extends LinearOpMode {
    private FtcDashboard ftcDashboard;
    private Limelight3A limelight;
    private Servo testServo;
    LLResult result;
    double[] pythonOutputs;
    private DigitalChannel touchSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(30);
        limelight.start();

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchServo");

        testServo = hardwareMap.get(Servo.class, "testServo");
        testServo.setPosition(0);

        waitForStart();

        while (opModeIsActive()){
            result = limelight.getLatestResult();
            pythonOutputs = result.getPythonOutput();
            telemetry.addData("Python List", Arrays.toString(pythonOutputs));

            if (pythonOutputs != null && pythonOutputs.length > 0) {
                double angle = pythonOutputs[5];
                double servoPosition = Math.min(1, Math.max(0, Math.abs(angle) / 180));
                testServo.setPosition(servoPosition);

                telemetry.addData("Raw Angle", angle);
                telemetry.addData("Servo Position", servoPosition);
            } else {
                telemetry.addLine("Invalid Python Output");
            }
            telemetry.addLine();
            telemetry.addData("Running", limelight.isRunning());
            telemetry.addData("Connected", limelight.isConnected());
            telemetry.addLine();
            if (touchSensor.getState() == false) {
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }
            telemetry.update();
        }
    }
}
