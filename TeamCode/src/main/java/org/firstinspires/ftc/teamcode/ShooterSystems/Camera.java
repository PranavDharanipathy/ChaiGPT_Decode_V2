package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Camera {

    private final Limelight3A limelight;

    public Limelight3A ll() {
        return limelight;
    }

    public Camera(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public Camera(HardwareMap hardwareMap, String limelightDeviceName) {
        limelight = hardwareMap.get(Limelight3A.class, limelightDeviceName);
    }

    public void start() {
        limelight.start();
    }

    public void setPollRateHz(int hz) {
        limelight.setPollRateHz(hz);
    }

    public void pipelineSwitch(int index) {
        limelight.pipelineSwitch(index);
    }

    public void reloadPipeline() {
        limelight.reloadPipeline();
    }

    public Pose getBotPoseFromTag(Goal.AprilTagCoordinates aprilTagCoord) {

        LLResult llResult = limelight.getLatestResult();

        if (!limelight.isConnected() || llResult == null || !llResult.isValid()) return null;

        LLResultTypes.FiducialResult tag = llResult.getFiducialResults().get(0);

        Pose3D llPose = tag.getCameraPoseTargetSpace();
        Position llPosition = llPose.getPosition();

        double cameraX = MathUtil.metersToInches(llPosition.x);
        double cameraZ = MathUtil.metersToInches(llPosition.z);

        double heading = llPose.getOrientation().getYaw(AngleUnit.RADIANS);

        return transformToField(new Pose(cameraX, cameraZ, heading), aprilTagCoord);
    }

    private Pose transformToField(Pose cameraPose, Goal.AprilTagCoordinates aprilTagCoord) {

        Goal.AprilTagCoordinate tag = aprilTagCoord.getAprilTagCoordinate();

        double fieldX = tag.getX() + cameraPose.getY() * Math.cos(tag.getYaw()) + cameraPose.getX() * Math.sin(tag.getYaw());
        double fieldY = tag.getY() + cameraPose.getY() * Math.sin(tag.getYaw()) - cameraPose.getX() * Math.cos(tag.getYaw());

        return new Pose(
                fieldX,
                fieldY,
                cameraPose.getHeading()
        );
    }
}
