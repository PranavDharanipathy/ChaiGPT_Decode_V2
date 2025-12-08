package org.firstinspires.ftc.teamcode.Auton;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutoRedFar extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        //follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain firstintake;
        public PathChain backtogoal;
        public PathChain Secondintake;
        public PathChain secondgoal;
        public PathChain thirdIntake;
        public PathChain thirdGoal;

        public Paths(Follower follower) {
            firstintake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.049, 8.976),
                                    new Pose(49.775, 38.567),
                                    new Pose(37.331, 36.078),
                                    new Pose(11.122, 35.707)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            backtogoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.122, 35.707),
                                    new Pose(50.537, 38.634),
                                    new Pose(62.439, 11.512)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Secondintake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.439, 11.512),
                                    new Pose(47.610, 70.634),
                                    new Pose(45.659, 58.146),
                                    new Pose(9.756, 60.098)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            secondgoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.756, 60.098),
                                    new Pose(39.024, 42.341),
                                    new Pose(55.415, 36.878),
                                    new Pose(65.171, 13.073)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            thirdIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(65.171, 13.073),
                                    new Pose(67.902, 92.293),
                                    new Pose(35.902, 84.878),
                                    new Pose(14.244, 83.902)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            thirdGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.244, 83.902),
                                    new Pose(12.683, 49.366),
                                    new Pose(50.927, 54.634),
                                    new Pose(64.585, 10.146)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {

        PathChain firstIntake = paths.firstintake;

        PathChain backtoGoal = paths.backtogoal;

        PathChain secondIntake = paths.Secondintake;

        PathChain secondGoal = paths.secondgoal;
        PathChain thirdIntake = paths.thirdIntake;
        PathChain thirdGoal = paths.thirdGoal;


        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}