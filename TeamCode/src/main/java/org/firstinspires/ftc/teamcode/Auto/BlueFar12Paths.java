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

        public BlueFar12Paths(Follower follower) {

            FirstIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(65.108, 7.832),
                                    new Pose(66.038, 40.572),
                                    new Pose(8.929, 35.363)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            FirstReturn = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.929, 35.363),
                                    new Pose(53.760, 32.759),
                                    new Pose(63.992, 10.622)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

           SecondIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.992, 10.622),
                                    new Pose(80.175, 64.941),
                                    new Pose(43.157, 72.568),
                                    new Pose(14.138, 45.594),
                                    new Pose(12.463, 68.847)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
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
                    .setTangentHeadingInterpolation()
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
