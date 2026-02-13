package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class BlueFar15Paths {

    private static final Logger log = LoggerFactory.getLogger(BlueFar15Paths.class);
    public PathChain FirstIntake;

    public PathChain FirstReturn;
    public PathChain SecondIntake;
    public PathChain IntakeExtra;
    public PathChain SecondReturn;

    public PathChain preload;
    public PathChain setupForFirstIntake;
    public PathChain intakeExtra;
    public PathChain firstReturnn;

    public PathChain SecondsetupForFirstIntake;

    public PathChain SecondintakeExtra;

    public PathChain SecondfirstReturnn;

    public BlueFar15Paths(Follower follower) {

        preload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(64, 9.5),
                                new Pose(64, 14.829))
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        FirstIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(64, 14.829), // y=14.829
                                new Pose(62, 18),
                                new Pose(59.049, 28.220),
                                new Pose(47.902, 38.829),
                                new Pose(19.415, 38.707)
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();


            /*FirstIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(65.780, 11.537),
                                    new Pose(42.146, 9.366),
                                    new Pose(56.585, 25.683),
                                    new Pose(42.537, 39.268),
                                    new Pose(7.220, 36.683)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();*/

        FirstReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.929, 38.363),
                                new Pose(50.760, 28.759),
                                new Pose(61.992, 18.622)
                        )
                )
                .setReversed()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(Math.PI)
                                )
                        )
                )
                //.setConstantHeadingInterpolation(Math.PI)

                .build();


        SecondIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(61.610, 18.000),
                                //new Pose(56.561, 34.268),
                                new Pose(55.390, 39.732),
                                //new Pose(49.707, 54.634),
                                new Pose(49, 49),
                                //new Pose(43.73200, 68), //y = 49
                                new Pose(31.244, 68),
                                new Pose(23.390, 68),
                                new Pose(19, 70)
                        )
                )
                //.setConstantHeadingInterpolation(Math.PI)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.35,
                                        HeadingInterpolator.facingPoint(22, 61)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.35,
                                        1,
                                        HeadingInterpolator.constant(Math.PI)
                                )
//                                    new HeadingInterpolator.PiecewiseNode(
//                                            0.87,
//                                            1,
//                                            HeadingInterpolator.linear(Math.PI, Math.toRadians(137))
//                                    )
                        )
                )
                .build();

        SecondReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.463, 68.847),
                                //new Pose(54.876, 45.408),

                                //y = 11 before
                                new Pose(65.154, 18.924)
                        )
                )
                // .setConstantHeadingInterpolation(Math.PI)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.085,
                                        HeadingInterpolator.constant(Math.PI)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.085,
                                        0.8,
                                        HeadingInterpolator.facingPoint(4, 60)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1,
                                        HeadingInterpolator.constant(Math.PI)
                                )
                        )
                )
                .build();

        setupForFirstIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(65.154, 18.924),
                                new Pose(151, 65).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.PI, Math.toRadians(270))
                .build();

        intakeExtra = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(151, 65).mirror(), new Pose(151, 21).mirror())
                )
                .setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        firstReturnn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(150, 26).mirror(),
                                new Pose(125, 30).mirror(),
                                new Pose(82, 11).mirror()
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.PI)
                .build();



        SecondsetupForFirstIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(65.154, 11.924),
                                new Pose(151, 65).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.PI, Math.toRadians(270))
                .build();

        SecondintakeExtra = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(151, 65).mirror(), new Pose(151, 21).mirror())
                )
                .setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        SecondfirstReturnn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(150, 26).mirror(),
                                new Pose(125, 30).mirror(),
                                new Pose(82, 11).mirror()
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.PI)
                .build();





    }
}