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
public class BlueFar12Paths {

    private static final Logger log = LoggerFactory.getLogger(BlueFar12Paths.class);
    public PathChain FirstIntake;
        public PathChain FirstReturn;
        public PathChain SecondIntake;
        public PathChain OpenGate;
        public PathChain IntakeExtra;
        public PathChain SecondReturn;

        public PathChain ThirdReturn;

        public PathChain preload;

        public BlueFar12Paths(Follower follower) {

             preload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(64, 9.5),
                                    new Pose(64, 14.829)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();

            FirstIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(64, 9.5), // y=14.829
                                    new Pose(62, 18),
                                    new Pose(59.049, 28.220),
                                    new Pose(47.902, 36.829),
                                    new Pose(19.415, 36.707)
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
                                    new Pose(8.929, 35.363),
                                    new Pose(50.760, 32.759),
                                    new Pose(63.992, 15.622)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();


            SecondIntake = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.610, 16.000),
                                    //new Pose(56.561, 34.268),
                                    new Pose(50.390, 39.732),
                                    //new Pose(49.707, 54.634),
                                    new Pose(42, 48),
                                    new Pose(39.732, 62), //y = 49
                                    new Pose(31.244, 62),
                                    new Pose(23.390, 62),
                                    new Pose(15, 63)
                            )
                    )
                    //.setConstantHeadingInterpolation(Math.PI)
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(
                                            0,
                                            0.4,
                                            HeadingInterpolator.facingPoint(23, 62)
                                    ),
                                    new HeadingInterpolator.PiecewiseNode(
                                            0.4,
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
                                    new Pose(65.154, 11.924)
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

            IntakeExtra = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(65.463, 11.924),
                                    new Pose(32, 11.924),
                                    new Pose(14.436, 11.924)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)

                    .setGlobalDeceleration()
                    .build();

            ThirdReturn = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(3.436, 0.390),
                                    new Pose(54.132, 29.597),
                                    new Pose(62.503, 8.762)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();





        }
    }
