package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.log;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@Configurable // Panels
@Config
public class RedFar12Paths {

    private static final Logger log = LoggerFactory.getLogger(RedFar12Paths.class);
    public PathChain FirstIntake;
    public PathChain FirstReturn;
    public PathChain SecondIntake;
    public PathChain OpenGate;
    public PathChain IntakeExtra;
    public PathChain SecondReturn;

    public PathChain ThirdReturn;

    public PathChain preload;
    public PathChain curvedIntake2;

    public PathChain setupForFirstIntake;
    public PathChain firstIntake;
    public PathChain firstReturnn;

    public RedFar12Paths(Follower follower) {

        preload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(64, 9.5).mirror(),
                                new Pose(64, 14.829).mirror()
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        FirstIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(64, 9.5).mirror(), //y=14.829
                                new Pose(62, 18).mirror(),
                                new Pose(59.049, 28.220).mirror(),
                                new Pose(47.902, 36.829).mirror(),
                                new Pose(19.415, 36.707).mirror()
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

        FirstReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.929, 35.363).mirror(),
                                new Pose(50.760, 32.759).mirror(),
                                new Pose(63.992, 15.622).mirror()
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();


        SecondIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(63.610, 16.000).mirror(),
                                //new Pose(56.561, 34.268),
                                new Pose(54.390, 39.732).mirror(),
                                //new Pose(49.707, 54.634),
                                new Pose(47, 54).mirror(),
                                new Pose(38.732, 62).mirror(), //y = 49
                                new Pose(25.244, 62).mirror(),
                                new Pose(13.390, 62).mirror(),
                                new Pose(13, 64).mirror()
                        )

                )

                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.4,
                                        HeadingInterpolator.facingPoint(145, 64)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.4,
                                        1,
                                        HeadingInterpolator.tangent
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
                        new BezierCurve(
                                new Pose(17.463, 51.847).mirror(),
                                new Pose(54.876, 35.408).mirror(),
                                new Pose(67.154, 11.924).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(25), 0)




                .build();
        ThirdReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(3.436, 0.390).mirror(),
                                new Pose(54.132, 29.597).mirror(),
                                new Pose(62.503, 8.762).mirror()
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();


        setupForFirstIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(97.000, 9.500).mirror(),
                                new Pose(151, 42).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(Math.toRadians(270)))
                .build();

        IntakeExtra = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(151, 42).mirror(), new Pose(151, 33).mirror())
                )
                .setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        firstReturnn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(150, 26).mirror(),
                                new Pose(125, 30).mirror(),
                                new Pose(97, 11).mirror()
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .build();






    }
}