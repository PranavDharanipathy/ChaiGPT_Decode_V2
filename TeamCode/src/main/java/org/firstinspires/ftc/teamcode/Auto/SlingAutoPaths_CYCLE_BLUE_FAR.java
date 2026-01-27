package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class SlingAutoPaths_CYCLE_BLUE_FAR {

    public Pose startPose;

    public PathChain setupForFirstIntake;
    public PathChain firstIntake;
    public PathChain firstReturnn;
    public PathChain intake;
    public PathChain returnn;

    public SlingAutoPaths_CYCLE_BLUE_FAR(Follower f) {

        startPose = new Pose(42, 9.5, Math.PI);

        setupForFirstIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(97.000, 9.500).mirror(),
                                new Pose(151, 42).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        firstIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(151, 42).mirror(), new Pose(151, 33).mirror())
                )
                .setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        firstReturnn = f
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

        intake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97, 11).mirror(), new Pose(130, 10.5).mirror())
                )
                .setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        returnn = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(136, 10.5).mirror(), new Pose(97, 11).mirror())
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}
