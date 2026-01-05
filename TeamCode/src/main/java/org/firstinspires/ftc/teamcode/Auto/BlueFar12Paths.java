package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


@Configurable // Panels
@Config
public class BlueFar12Paths {

        public PathChain FirstIntake;
        public PathChain FirstReturn;
        public PathChain SecondIntake;
        public PathChain OpenGate;
        public PathChain IntakeExtra;
        public PathChain SecondReturn;

        public PathChain ThirdReturn;

        public PathChain Preload;

        public BlueFar12Paths(Follower follower) {

            /* preload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(64.976, 8.195),
                                    new Pose(64.976, 16.585),
                                    new Pose(59.317, 18.927)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();*/

            FirstIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(64.195, 14.829),
                                    new Pose(59.049, 31.220),
                                    new Pose(43.902, 31.829),
                                    new Pose(14.415, 31.707)
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
                                    new Pose(8.929, 35.363),
                                    new Pose(50.760, 32.759),
                                    new Pose(63.992, 15.622)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();

            SecondIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.634, 14.244),
                                    new Pose(55.220, 68.488),
                                    new Pose(42.390, 55.537),
                                    new Pose(16.659, 55.488)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SecondReturn = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.463, 68.847),
                                    new Pose(54.876, 45.408),
                                    new Pose(67.154, 11.924)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();

            IntakeExtra = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.463, 68.847),
                                    new Pose(3.162, 39.270),
                                    new Pose(3.436, 0.390)
                            )
                    )
                    .setTangentHeadingInterpolation()
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
                    .setTangentHeadingInterpolation()
                    .build();





        }
    }
