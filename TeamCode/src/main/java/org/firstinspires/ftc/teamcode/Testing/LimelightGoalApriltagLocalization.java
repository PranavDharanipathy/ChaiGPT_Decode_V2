package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.PIPELINES;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Autonomous (group = "testing")
public class LimelightGoalApriltagLocalization extends OpMode {

    private Limelight3A limelight;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, Constants.MapSetterConstants.limelight3AUSBDeviceName);
    }

    @Override
    public void start() {

        limelight.setPollRateHz(ShooterInformation.CameraConstants.CAMERA_POLL_RATE);
        limelight.start();
        limelight.pipelineSwitch(PIPELINES.BLUE_PIPELINE.getPipelineIndex());
    }

    @Override
    public void loop() {

        LLStatus llStatus = limelight.getStatus();

        telemetry.addData("is connected", limelight.isConnected());
        telemetry.addData("is running", limelight.isRunning());
        telemetry.addData("fps", llStatus.getFps());
        telemetry.addData("cpu usage", llStatus.getCpu());

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            LLResultTypes.FiducialResult tag = llResult.getFiducialResults().get(0);

            Pose3D llPose3d = tag.getCameraPoseTargetSpace();
            Position llPosition = llPose3d.getPosition();

            double x = MathUtil.metersToInches(llPosition.x);
            double y = MathUtil.metersToInches(llPosition.y);
            double z = MathUtil.metersToInches(llPosition.z);

            String orientation = llPose3d.getOrientation().toString();

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("z", z);
            telemetry.addData("orientation", orientation);
        }
        else {
            telemetry.addLine("no ll data");
        }

        telemetry.update();
    }
}
