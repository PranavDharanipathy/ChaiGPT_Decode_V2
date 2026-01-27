package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseVelocityTracker;
import org.firstinspires.ftc.teamcode.util.Rev9AxisImuWrapped;

@TeleOp(group = "testing")
public class PoseVelocityTrackerTesting extends TeleOpBaseOpMode {

    private PoseVelocityTracker poseVelocityTracker;

    @Override
    public void runOpMode() {

        initializeDevices();
        applyComponentTraits();

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Rev9AxisImuWrapped rev9AxisImuWrapped = new Rev9AxisImuWrapped(rev9AxisImu);

        poseVelocityTracker = new PoseVelocityTracker(follower, rev9AxisImuWrapped);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            poseVelocityTracker.update();

            Pose botPose = follower.getPose();

            PoseVelocity botVel = poseVelocityTracker.getPoseVelocity();

            telemetry.addData("raw pose", botPose.toString());
            telemetry.addData("clean pose", ShooterInformation.Calculator.getBotPose(botPose, rev9AxisImuWrapped.getYaw()));

            telemetry.addData("future pose",
                    ShooterInformation.Calculator.getFutureRobotPose(
                            1.25,
                            botPose,
                            botVel
                    ).toString()
            );

            telemetry.addData("bot vel", "x: %.2f, y: %.2f, heading: %.2f", botVel.getXVelocity(), botVel.getYVelocity(), botVel.getAngularVelocity());

            telemetry.update();
        }
    }
}
