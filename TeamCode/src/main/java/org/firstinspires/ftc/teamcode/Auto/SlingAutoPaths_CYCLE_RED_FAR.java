package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
                        new BezierLine(new Pose(97.000, 9.500), new Pose(145, 26.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        firstIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(145, 26.000), new Pose(145, 12))
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        firstReturnn = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(145, 12), new Pose(97, 16))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        intake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97, 16), new Pose(145, 16))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        returnn = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(145, 16), new Pose(97, 16))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
