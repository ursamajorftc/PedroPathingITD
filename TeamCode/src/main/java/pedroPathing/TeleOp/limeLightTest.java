package pedroPathing.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Limelight", group = "Linear OpMode")
public class limeLightTest extends LinearOpMode {
    private FtcDashboard ftcDashboard;
    private Limelight3A limelight;
    private Servo testServo;
    private LLResult result;
    private double[] block_data;
    private double angle;



    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        testServo = hardwareMap.get(Servo.class, "testServo");

        waitForStart();

        while (opModeIsActive()){
            result = limelight.getLatestResult();
            block_data = result.getPythonOutput();
            angle = block_data[5];
            testServo.setPosition(Math.abs(angle)/180);
        }
    }
}
