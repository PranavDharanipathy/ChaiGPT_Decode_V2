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
    public PathChain curvedIntake1;
    public PathChain curvedReturn1;
    public PathChain curvedIntake2;
    public PathChain curvedReturn2;
    public PathChain normalIntake;
    public PathChain normalReturn;

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
                                new Pose(125, 33),
                                new Pose(97, 11)
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        curvedIntake1 = f
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(97, 11),
                                new Pose(115, 36),
                                new Pose(136, 26))
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-65))
                .build();

        curvedReturn1 = f
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(136, 26),
                                new Pose(115, 36),
                                new Pose(97, 11)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-65), Math.toRadians(0))
                .build();

        curvedIntake2 = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97, 11), new Pose(135, 14)))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-35))
                .build();

        curvedReturn2 = f
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135, 14),
                                new Pose(115, 19),
                                new Pose(97, 11)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.5,
                                        HeadingInterpolator.linear(Math.toRadians(-35), Math.toRadians(0))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.5,
                                        1,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                )
                        )
                )
                .build();

        normalIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97, 11), new Pose(136, 10.5))
                )
                .setNoDeceleration()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.linear(Math.toRadians(0), Math.toRadians(-20))
                                )
                        )
                )
                .build();

        normalReturn = f
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(136, 10.5), new Pose(97, 11))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
