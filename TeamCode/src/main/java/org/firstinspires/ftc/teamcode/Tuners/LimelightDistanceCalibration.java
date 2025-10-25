package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;

@Config
@TeleOp (group = "tuning")
public class LimelightDistanceCalibration extends OpMode {

    private Limelight3A limelight;

    public static int PIPELINE;

    private Telemetry telemetry;

    /*
    * LATEST TUNED REGRESSION
    * distance = 0.34197 * ty^2 - 3.79725ty + 53.01088
    * */

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, Constants.MapSetterConstants.limelight3AUSBDeviceName);

        limelight.setPollRateHz(ShooterInformation.CameraConstants.CAMERA_POLL_RATE);
        limelight.pipelineSwitch(PIPELINE);
    }

    @Override
    public void loop() {

        Double ty;

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            ty = result.getTy();
        }
        else ty = null;

        telemetry.addData("ty", ty);
        telemetry.update();
    }
}
