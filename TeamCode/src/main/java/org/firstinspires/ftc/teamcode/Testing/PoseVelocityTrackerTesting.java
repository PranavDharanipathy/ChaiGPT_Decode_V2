package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
@TeleOp(group = "testing")
public class PoseVelocityTrackerTesting extends TeleOpBaseOpMode {

    private PoseVelocityTracker poseVelocityTracker;

    @Override
    public void runOpMode() {

        initializeDevices();
        applyComponentTraits();

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        poseVelocityTracker = new PoseVelocityTracker(follower);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            poseVelocityTracker.update();

            Pose botPose = follower.getPose();

            PoseVelocity botVel = poseVelocityTracker.getPoseVelocity();

            telemetry.addData("raw pose", botPose.toString());

            telemetry.addData("future pose",
                    ShooterInformation.Calculator.getFutureRobotPose(
                            1.3,
                            botPose,
                            botVel
                    ).toString()
            );

            telemetry.addData("bot vel", "x: %.2f, y: %.2f, heading: %.2f", botVel.getXVelocity(), botVel.getYVelocity(), botVel.getAngularVelocity());

            double[][] histories = poseVelocityTracker.getHistories();
            double[] xVelHistory = histories[0];
            double[] yVelHistory = histories[1];
            double[] angVelHistory = histories[2];

            telemetry.addData("xVelHistory", "P:%.2f, C:%.2f", xVelHistory[0], xVelHistory[1]);
            telemetry.addData("yVelHistory", "P:%.2f, C:%.2f", yVelHistory[0], yVelHistory[1]);
            telemetry.addData("angVelHistory", "P:%.2f, C:%.2f", angVelHistory[0], angVelHistory[1]);

            telemetry.update();
        }
    }
}
