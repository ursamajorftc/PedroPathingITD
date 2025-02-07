package pedroPathing.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.examples.iHopeSpecsWorkLol1;

@Autonomous(name = "i hope specs work lol", group = "Examples")
public class iHopeSpecsWorkLol extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private FtcDashboard dashboard;
    private int pathState;
    private final Pose startPose = new Pose(20.331, 24.700, Math.toRadians(180));



    private Path scorePreload, park;
    private PathChain push1, push2pickUp, score1, pickUp2, score2, pickUp3, score3;

    public void buildPaths() {

        scorePreload = new Path(
                new BezierCurve(
                        new Point(20.331, 24.700, Point.CARTESIAN),
                        new Point(115.099, 14.282, Point.CARTESIAN),
                        new Point(20.331, 14.282, Point.CARTESIAN)
                ));

        scorePreload.setConstantHeadingInterpolation(Math.toRadians(180));


        push1 = follower.pathBuilder().addPath(
                // Line 2
                new BezierCurve(
                        new Point(39.319, 72.588, Point.CARTESIAN),
                        new Point(7.057, 0.672, Point.CARTESIAN),
                        new Point(97.288, 57.970, Point.CARTESIAN),
                        new Point(44.863, 32.261, Point.CARTESIAN),
                        new Point(65.363, 20.499, Point.CARTESIAN),
                        new Point(69.900, 26.044, Point.CARTESIAN),
                        new Point(20.331, 24.700, Point.CARTESIAN)
                )
        )
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();



    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()){
                   // follower.followPath(push1,true);
                  //  setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){

                }
                break;


        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }


}
