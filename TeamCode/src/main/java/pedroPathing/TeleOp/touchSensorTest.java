package pedroPathing.TeleOp;

import android.text.method.Touch;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@TeleOp(name = "Touch Sensor", group = "Linear OpMode")
public class touchSensorTest extends LinearOpMode {
    TouchSensor touchSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        waitForStart();

        while(opModeIsActive()){
           telemetry.addData("ispressed", touchSensor.isPressed());

            telemetry.update();
        }

    }
}
