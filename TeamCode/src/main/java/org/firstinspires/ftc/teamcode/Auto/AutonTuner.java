package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;

public class AutonTuner extends OpMode {

    public static double goX = 64;
    public static double goY = 9.5;


    Follower follower;

    PinpointLocalizer localizer;



    @Override
    public void init() {
        localizer = new PinpointLocalizer(hardwareMap, PPConstants.localizerConstants, new Pose(64, 9.5, 0));
        follower = PPConstants.createFollower(hardwareMap);

    }

    double currX = 0;
    double currY = 0;

    @Override
    public void loop() {

        currX = localizer.getPose().getX();
        currY = localizer.getPose().getY();

            /*follower.followPath(
                    new BezierLine(
                    new Pose(currX, currY),
                    new Pose(goX, goY)

                    )

            )*/



    }
}