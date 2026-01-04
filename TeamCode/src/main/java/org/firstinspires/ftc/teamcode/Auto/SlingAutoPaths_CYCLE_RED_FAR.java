package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class SlingAutoPaths_CYCLE_RED_FAR {

    public Pose startPose;

    public PathChain setupForFirstIntake;
    public PathChain firstIntake;
    public PathChain firstReturnn;
    public PathChain intake;
    public PathChain returnn;

    public SlingAutoPaths_CYCLE_RED_FAR(Follower f) {

        startPose = new Pose(97, 9.5, 0);

        setupForFirstIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.000, 9.500), new Pose(151, 42))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        firstIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(151, 42), new Pose(151, 33))
                )
                .setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        firstReturnn = f
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(150, 26),
                                new Pose(125, 30),
                                new Pose(97, 11)
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        intake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97, 11), new Pose(136, 10.5))
                )
                .setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnn = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(136, 10.5), new Pose(97, 11))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
