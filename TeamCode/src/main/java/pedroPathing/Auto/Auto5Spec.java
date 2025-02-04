//package pedroPathing.Auto;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathBuilder;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import pedroPathing.pid.*;
//import pedroPathing.constants.*;
//public class spec5path {
//
//
//    public spec5path() {
//        PathBuilder builder = new PathBuilder();
//
//
//        builder
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(8.414, 56.494, Point.CARTESIAN),
//                                new Point(36.421, 65.269, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 2
//                        new BezierCurve(
//                                new Point(36.421, 65.269, Point.CARTESIAN),
//                                new Point(19.472, 10.457, Point.CARTESIAN),
//                                new Point(77.048, 57.095, Point.CARTESIAN),
//                                new Point(60.341, 22.838, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 3
//                        new BezierLine(
//                                new Point(60.341, 22.838, Point.CARTESIAN),
//                                new Point(21.155, 22.718, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 4
//                        new BezierCurve(
//                                new Point(21.155, 22.718, Point.CARTESIAN),
//                                new Point(53.730, 37.262, Point.CARTESIAN),
//                                new Point(58.658, 13.102, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(58.658, 13.102, Point.CARTESIAN),
//                                new Point(20.915, 13.342, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 6
//                        new BezierLine(
//                                new Point(20.915, 13.342, Point.CARTESIAN),
//                                new Point(14.905, 33.656, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 7
//                        new BezierLine(
//                                new Point(14.905, 33.656, Point.CARTESIAN),
//                                new Point(35.579, 68.033, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 8
//                        new BezierLine(
//                                new Point(35.579, 68.033, Point.CARTESIAN),
//                                new Point(14.905, 33.656, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 9
//                        new BezierLine(
//                                new Point(14.905, 33.656, Point.CARTESIAN),
//                                new Point(35.579, 68.154, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 10
//                        new BezierLine(
//                                new Point(35.579, 68.154, Point.CARTESIAN),
//                                new Point(14.905, 33.896, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 11
//                        new BezierLine(
//                                new Point(14.905, 33.896, Point.CARTESIAN),
//                                new Point(35.459, 68.154, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 12
//                        new BezierLine(
//                                new Point(35.459, 68.154, Point.CARTESIAN),
//                                new Point(14.785, 33.776, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .addPath(
//                        // Line 13
//                        new BezierLine(
//                                new Point(14.785, 33.776, Point.CARTESIAN),
//                                new Point(35.579, 68.033, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
//    }
//}
//
//
//
