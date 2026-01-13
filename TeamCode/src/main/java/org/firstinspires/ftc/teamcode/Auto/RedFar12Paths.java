package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


@Configurable // Panels
@Config
public class RedFar12Paths {

    public PathChain FirstIntake;
    public PathChain FirstReturn;
    public PathChain SecondIntake;
    public PathChain OpenGate;
    public PathChain IntakeExtra;
    public PathChain SecondReturn;

    public PathChain ThirdReturn;

    public PathChain preload;

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
                                new Pose(64, 14.829).mirror(),
                                new Pose(59.049, 31.220).mirror(),
                                new Pose(43.902, 31.829).mirror(),
                                new Pose(19.415, 31.707).mirror()
                        )
                )
                .setTangentHeadingInterpolation()
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
                                new Pose(54.390, 36.732).mirror(),
                                //new Pose(49.707, 54.634),
                                new Pose(47, 46).mirror(),
                                new Pose(38.732, 51.488).mirror(), //y = 49
                                new Pose(25.244, 51.561).mirror(),
                                new Pose(13.390, 65.00000).mirror()
                        )
                )
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();


        SecondReturn = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.463, 68.847).mirror(),
                                new Pose(54.876, 45.408).mirror(),
                                new Pose(67.154, 11.924).mirror()
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        IntakeExtra = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.463, 68.847).mirror(),
                                new Pose(3.162, 39.270).mirror(),
                                new Pose(3.436, 0.390).mirror()
                        )
                )
                .setTangentHeadingInterpolation()
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
                .setTangentHeadingInterpolation()
                .build();





    }
}
