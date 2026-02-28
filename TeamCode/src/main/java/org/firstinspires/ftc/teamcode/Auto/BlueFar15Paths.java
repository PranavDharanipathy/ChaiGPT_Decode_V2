package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;


public class BlueFar15Paths {

    public PathChain SecondIntake;

    public PathChain SecondReturn;
    public PathChain FirstIntake;
    public PathChain FirstReturn;



    public PathChain preload;
    public PathChain setupForFirstIntake;
    public PathChain intakeExtra;
    public PathChain firstReturnn;

    public PathChain hpIntake;
    public PathChain hpReturn;

    public PathChain movementRP;
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


        SecondIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(64, 9.5), // y=14.829
                                new Pose(62, 18),
                                new Pose(59.049, 27),
                                new Pose(47.902, 30),
                                new Pose(19.415, 29.707)
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


        SecondReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.929, 31.363),
                                new Pose(50.760, 28.759),
                                new Pose(61.992, 12.622)
                        )
                )

                .setConstantHeadingInterpolation(Math.PI)

                .build();




        FirstIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(61.610, 12.000),
                                //new Pose(49.707, 54.634),
                                new Pose(59, 46.5),
                                //new Pose(43.73200, 68), //y = 49
                                new Pose(53.24,4, 28),
                                new Pose(47, 38),
                                new Pose(19, 38)
                        )
                )
                //.setConstantHeadingInterpolation(Math.PI)
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.45,
                                        HeadingInterpolator.facingPoint(22, 59)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.45,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(165))
                                )
                        )
                )
                .build();


        /*FirstReturn = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(18, 58),
                        new Pose(64, 11.924)
                )

        ).setHeadingInterpolation(
                HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                0.8,
                                HeadingInterpolator.constant(Math.toRadians(145))
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                0.8,
                                1,
                                HeadingInterpolator.constant(Math.PI)
                        )

        )
        ).build(); */


        FirstReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.929, 38.363),
                                new Pose(50.760, 28.759),
                                new Pose(61.992, 17.622)
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();

                /*.setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.2,
                                        HeadingInterpolator.constant(Math.toRadians(145))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.2,
                                        0.8,
                                        HeadingInterpolator.constant(Math.toRadians(170))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1,
                                        HeadingInterpolator.constant(Math.PI)
                                )
                        )
                )
                .build();*/


        setupForFirstIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(65.154, 11.924),
                                new Pose(151, 65).mirror()
                        )
                )

                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.05,
                                        HeadingInterpolator.constant(Math.PI)
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.05,
                                        0.55,
                                        HeadingInterpolator.linear(Math.PI, Math.toRadians(270))
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.55,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(270))
                                )
                        )
                )
                //.setLinearHeadingInterpolation(Math.PI, Math.toRadians(270))
                .build();


        /*hpIntake= follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(-4, 54/*60),

                                new Pose(-4, 11)
                        )
                )*/


        intakeExtra = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(151, 65).mirror(), new Pose(151, 21).mirror())
                )
                .setNoDeceleration()

                .setConstantHeadingInterpolation(Math.toRadians(270)
                /*.setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.constant(Math.PI)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.linear(Math.PI, Math.toRadians(270))
                                )
                        )*/
                )
                .build();

        hpReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(-4, 11),
                                new Pose(64, 16.5).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();



        intakeExtra = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(64, 12.5).mirror(),
                                new Pose(12, 12.5)
                        )
                )
                //.setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();


        firstReturnn = follower
                .pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18, 12.5),
                                new Pose(125, 30).mirror(),
                                new Pose(92, 17).mirror()
                        )
                )
                .setNoDeceleration()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.10,
                                        HeadingInterpolator.facingPoint(0, 8)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.10, 1,
                                        HeadingInterpolator.constant(Math.PI)
                                )
                        )
                )
                //.setLinearHeadingInterpolation(Math.toRadians(270), Math.PI)
                .build();

        movementRP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(43, 9.5),
                                new Pose(34, 16)
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();

    }
}
