package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Peak
public class CalibratedVision {

    private final Limelight3A limelight;

    public CalibratedVision(Limelight3A limelight3A) {

        limelight = limelight3A;

        limelight.setPollRateHz(ShooterInformation.CameraConstants.CAMERA_POLL_RATE);
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public void start() {
        limelight.start();
    }

    private LLResult result;

    private void legalLLResultProcesses() {


    }

    public boolean update() {

        boolean llresultValidity;

        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            llresultValidity = true;

            legalLLResultProcesses();
        }
        else {
            llresultValidity = false;
        }

        return llresultValidity;
    }

    public void setPipeline(PIPELINES pipeline) {

        limelight.pipelineSwitch(pipeline.getPipelineIndex());
    }

    public int getCurrentPipeline() {
        return result.getPipelineIndex();
    }
}
