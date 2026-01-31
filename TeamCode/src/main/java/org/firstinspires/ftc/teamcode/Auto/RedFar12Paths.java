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

import org.firstinspires.ftc.teamcode.TeleOp.Intake;
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
                                new Pose(61.610, 18.000).mirror(),
                                //new Pose(56.561, 34.268).mirror(),
                                new Pose(55.390, 39.732).mirror(),
                                //new Pose(49.707, 54.634).mirror(),
                                new Pose(49, 49).mirror(),
                                //new Pose(43.73200, 68).mirror(), //y = 49
                                new Pose(31.244, 68).mirror(),
                                new Pose(23.390, 68).mirror(),
                                new Pose(19, 70).mirror()
                        )
                )
                /*.addPath(
                        new BezierCurve(
                                new Pose(63.610, 16.000).mirror(),
                                //new Pose(56.561, 34.268),
                                new Pose(54.390, 37).mirror(),
                                //new Pose(49.707, 54.634),
                                //new Pose(47, 54).mirror(),
                                new Pose(38.732, 55).mirror(), //y = 49
                                new Pose(25.244, 55).mirror(),
                                new Pose(13.390, 55).mirror(),
                                new Pose(13, 57).mirror()
                        )

                )*/
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.35,
                                        HeadingInterpolator.facingPoint(151, 61)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.35,
                                        1,
                                        HeadingInterpolator.constant(0)
                                )
//                                    new HeadingInterpolator.PiecewiseNode(
//                                            0.87,
//                                            1,
//                                            HeadingInterpolator.linear(Math.PI, Math.toRadians(137))
//                                    )
                        )
                )

                /*.setHeadingInterpolation(
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
                        )
                )*/
                .build();

        SecondReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(17.463, 51.847).mirror(),
                                new Pose(54.876, 35.408).mirror(),
                                new Pose(59.154, 14.924).mirror()
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
                                new Pose(58.503, 8.762).mirror()
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();


        setupForFirstIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.503, 8.762).mirror(), new Pose(151, 65))
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(

                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.21,
                                        HeadingInterpolator.tangent
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.21, 0.29,
                                        HeadingInterpolator.facingPoint(158,0)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.29, 1,
                                        HeadingInterpolator.constant(Math.toRadians(270))
                                )


                        )
                )
                //.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        IntakeExtra = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(151, 42),
                        new Pose(151, 33)
                        )
                )

                .setNoDeceleration()

                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        firstReturnn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(150, 26),
                                new Pose(125, 30),
                                new Pose(86, 11)
                        )
                )
                .setNoDeceleration()

                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0, 0.23,
                                        HeadingInterpolator.linear(Math.toRadians(270), 0)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.23, 1,
                                        HeadingInterpolator.constant(0)
                                )
                        )
                )
                //.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .build();






    }
}