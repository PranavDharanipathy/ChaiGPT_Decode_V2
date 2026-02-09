package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

public class SlingAutoPaths_CYCLE_BLUE_FAR {

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

    public PathChain movementRP;

    public SlingAutoPaths_CYCLE_BLUE_FAR(Follower f) {

        startPose = new Pose(97, 9.5, 0).mirror();

        setupForFirstIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(97.000, 9.500).mirror(),
                                new Pose(151, 42).mirror()
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.11,
                                        HeadingInterpolator.constant(Math.PI)
                                ),

                                new HeadingInterpolator.PiecewiseNode(
                                        0.11,
                                        1,
                                        HeadingInterpolator.linear(Math.PI, Math.toRadians(270))
                                )
                        )
                )
                //.setLinearHeadingInterpolation(Math.PI, Math.toRadians(270))
                .build();

        firstIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(151, 42).mirror(),
                                new Pose(151, 33).mirror()
                        )
                )
                .setNoDeceleration()
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        firstReturnn = f
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(150, 26).mirror(),
                                new Pose(125, 33).mirror(),
                                new Pose(97, 12.5).mirror()
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.PI)
                .build();

        curvedIntake1 = f
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(97, 12.5).mirror(),
                                new Pose(115, 36).mirror(),
                                new Pose(136, 26).mirror()
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.PI, Math.toRadians(245))
                .build();

        curvedReturn1 = f
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(136, 26).mirror(),
                                new Pose(115, 36).mirror(),
                                new Pose(97, 12.5).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.PI)
                .build();

        curvedIntake2 = f
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(97, 12.5).mirror(),
                                new Pose(135, 14).mirror()
                        )
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(Math.PI, Math.toRadians(215))
                .build();

        curvedReturn2 = f
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135, 14).mirror(),
                                new Pose(115, 19).mirror(),
                                new Pose(97, 12.5).mirror()
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.5,
                                        HeadingInterpolator.linear(Math.toRadians(215), Math.PI)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.5,
                                        1,
                                        HeadingInterpolator.constant(Math.PI)
                                )
                        )
                )
                .build();

        normalIntake = f
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(97, 12.5).mirror(),
                                new Pose(136, 10.5).mirror()
                        )
                )
                .setNoDeceleration()
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.7,
                                        HeadingInterpolator.constant(Math.PI)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.linear(Math.PI, Math.toRadians(200))
                                )
                        )
                )
                .build();

        normalReturn = f
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(136, 10.5).mirror(),
                                new Pose(97, 12.5).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        movementRP = f
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(64, 9.5),
                                new Pose(40, 16)
                        )
                )
                .setConstantHeadingInterpolation(Math.PI)
                .build();
    }
}
