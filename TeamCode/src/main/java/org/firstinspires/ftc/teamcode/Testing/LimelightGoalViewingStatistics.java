package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

@Peak
@Config
@TeleOp (group = "testing")
public class LimelightGoalViewingStatistics extends LinearOpMode {

    private Limelight3A limelight3A;

    /// camera poll rate Hz
    public static int CAMERA_POLL_RATE = 85;

    public static int PIPELINE = 2;

    private Telemetry telemetry;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight3A = hardwareMap.get(Limelight3A.class, Constants.MapSetterConstants.limelight3AUSBDeviceName);
        limelight3A.pipelineSwitch(PIPELINE);

        if (isStopRequested()) return;
        waitForStart();

        limelight3A.start();

        while (opModeIsActive()) {

            limelight3A.setPollRateHz(CAMERA_POLL_RATE);

            LLResult result = limelight3A.getLatestResult();

            if (result != null && result.isValid()) {

                if (!result.getFiducialResults().isEmpty()) {

                    int tagID = result.getFiducialResults().get(0).getFiducialId();
                    telemetry.addData("First Tag ID", tagID);

                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Avg Distance", result.getBotposeAvgDist());
                }
            }

            telemetry.addData("result.isValid()", result.isValid());
            telemetry.addData("limelight3A.isConnected()", limelight3A.isConnected());
            telemetry.update();
        }
    }

}
