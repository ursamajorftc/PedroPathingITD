package pedroPathing.Actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import pedroPathing.constants.*;
import android.view.View;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import pedroPathing.constants.*;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import pedroPathing.tuners_tests.pid.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;


public class Intake {
    private DcMotor intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
    private CRServo intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
    private CRServo intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
    private Servo intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
    private Servo intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");

    private static final double RIGHTINTAKEUP = 0.695;
    private static final double LEFTINTAKEUP = 0.32;
    private static final double RIGHTINTAKEDOWN = 0.45;
    private static final double LEFTINTAKEDOWN = 0.54;
    private double intakeTimer = 0;


    public void init(){
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Intake () {}
    public void setIntakePos(int distance) {
        intakeDrive.setTargetPosition(distance);
        intakeCRSLeft.setPower(-1);
        intakeCRSRight.setPower(1);
    }
    public void intakeDown(){
        intakeServoLeft.setPosition(LEFTINTAKEDOWN);
        intakeServoRight.setPosition(RIGHTINTAKEDOWN);
        intakeCRSLeft.setPower(-1);
        intakeCRSRight.setPower(1);
    }
    public void intakeUp(){
        intakeServoLeft.setPosition(LEFTINTAKEUP);
        intakeServoRight.setPosition(RIGHTINTAKEUP);
        intakeCRSLeft.setPower(-1);
        intakeCRSRight.setPower(1);
    }
    public double getIntakePos() {return intakeDrive.getCurrentPosition();}
}
