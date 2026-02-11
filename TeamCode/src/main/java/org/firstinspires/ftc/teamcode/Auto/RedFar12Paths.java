package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;


public class RedFar12Paths {

    public PathChain SecondIntake;

    public PathChain SecondReturn;
    public PathChain FirstIntake;
    public PathChain FirstReturn;



    public PathChain preload;
    public PathChain setupForFirstIntake;
    public PathChain intakeExtra;
    public PathChain firstReturnn;

    public PathChain movementRP;


    public RedFar12Paths(Follower follower) {


        preload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(64, 9.5).mirror(),
                                new Pose(64, 14.829).mirror())
                )
                .setConstantHeadingInterpolation(0)
                .build();


        SecondIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(64, 9.5).mirror(), // y=14.829
                                new Pose(62, 18).mirror(),
                                new Pose(59.049, 28).mirror(),
                                new Pose(47.902, 38).mirror(),
                                new Pose(19.415, 38.707).mirror()
                        )
                )
                .setConstantHeadingInterpolation(0)
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


        SecondReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.929, 38.363).mirror(),
                                new Pose(50.760, 28.759).mirror(),
                                new Pose(61.992, 18.622).mirror()
                        )
                )

                .setConstantHeadingInterpolation(0)

                .build();




        FirstIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(61.610, 18.000).mirror(),
                                //new Pose(56.561, 34.268).mirror(,
                                new Pose(55.390, 39.732).mirror(),
                                //new Pose(49.707, 54.634).mirror(,
                                new Pose(49, 42.5).mirror(),
                                //new Pose(43.73200, 68).mirror(, //y = 49
                                new Pose(31.244, 44).mirror(),
                                new Pose(24, 46).mirror(),
                                new Pose(13 /*15.5*/, 50).mirror()
                        )
                )
                //.setConstantHeadingInterpolation(Math.PI)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.45,
                                        HeadingInterpolator.facingPoint(new Pose(22, 61).mirror())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.45,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(35))
                                )
                        )
                )
                .build();


        FirstReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18, 55).mirror(),

                                new Pose(54, 36).mirror(),

                                //y = 11 before
                                new Pose(64, 18.924).mirror()
                        )
                )

                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.2,
                                        HeadingInterpolator.constant(Math.toRadians(38))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.2,
                                        0.8,
                                        HeadingInterpolator.constant(Math.toRadians(10))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1,
                                        HeadingInterpolator.constant(0)
                                )
                        )
                )
                .build();


        setupForFirstIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(65.154, 18.924).mirror(),
                                new Pose(148, 52/*60*/)
                        )
                )

                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.05,
                                        HeadingInterpolator.constant(0)
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.05,
                                        0.55,
                                        HeadingInterpolator.linear(0, Math.toRadians(-90))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.55,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(-90))
                                )
                        )
                )
                //.setLinearHeadingInterpolation(Math.PI, Math.toRadians(270))
                .build();


        intakeExtra = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(148, 65),
                                new Pose(147, 21)
                        )
                )
                //.setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();


        firstReturnn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(150, 26),
                                new Pose(125, 30),
                                new Pose(82, 11)
                        )
                )
                .setNoDeceleration()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.10,
                                        HeadingInterpolator.facingPoint(new Pose(0, 8).mirror())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.10, 1,
                                        HeadingInterpolator.constant(0)
                                )
                        )
                )
                //.setLinearHeadingInterpolation(Math.toRadians(270), Math.PI)
                .build();

        movementRP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(64, 9.5).mirror(),
                                new Pose(40, 16).mirror()
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

    }
}
