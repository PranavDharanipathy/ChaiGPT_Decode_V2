package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ShooterSystems.Camera;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseVelocityTracker;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.HashMap;

@SuppressWarnings("DataFlowIssue")
public class PoseEstimator {

    private CurrentAlliance.ALLIANCE alliance;
    private final Follower follower;
    private final PoseVelocityTracker poseVelocityTracker;
    private final Camera camera;

    private final HashMap<String, Double> covariance = new HashMap<>();
    private final HashMap<String, Double> processNoise = new HashMap<>();
    private final HashMap<String, Double> sensorNoice = new HashMap<>();

    //state stuff
    private double x = 0, y = 0, heading = 0;

    public PoseEstimator(Follower follower, Camera unstartedCamera, CurrentAlliance alliance) {

        this.alliance = alliance.getAlliance();

        this.follower = follower;
        poseVelocityTracker = new PoseVelocityTracker(follower);
        camera = unstartedCamera;

        covariance.put("Pxx", 1.0);
        covariance.put("Pyy", 1.0);
        covariance.put("Phh", 1.0);

        processNoise.put("Qx", 0.0);
        processNoise.put("Qy", 0.0);
        processNoise.put("Qh", 0.0);

        sensorNoice.put("Rodo", 0.0);
        sensorNoice.put("Rll", 0.0);
    }

    public void setKFParameters(double Qx, double Qy, double Qh, double Rodo, double Rll) {

        processNoise.replace("Qx", Qx);
        processNoise.replace("Qy", Qy);
        processNoise.replace("Qh", Qh);

        sensorNoice.replace("Rodo", Rodo);
        sensorNoice.replace("Rll", Rll);
    }

    public void restartKF() {

        covariance.put("Pxx", 1.0);
        covariance.put("Pyy", 1.0);
        covariance.put("Phh", 1.0);
    }

    public void start() {

        camera.setPollRateHz(ShooterInformation.CameraConstants.CAMERA_POLL_RATE);
        camera.start();
    }

    private Pose odoPose;
    private PoseVelocity odoVelocity;
    private Pose llPose;
    private Pose finalPose;

    private final ElapsedTime timer = new ElapsedTime();
    private double prevTime = 0, currTime = 0, dt;

    public void update() {

        prevTime = currTime;
        currTime = timer.seconds();
        dt = currTime - prevTime;

        follower.update();
        poseVelocityTracker.update();

        odoVelocity = poseVelocityTracker.getPoseVelocity();

        predict();

        odoPose = follower.getPose();
        llPose = camera.getBotPoseFromTag(alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? Goal.AprilTagCoordinates.BLUE : Goal.AprilTagCoordinates.RED);

        updateState(odoPose.getX(), odoPose.getY(), odoPose.getHeading(), sensorNoice.get("Rodo")); //odo
        updateState(llPose.getX(), llPose.getY(), llPose.getHeading(), sensorNoice.get("Rll")); //ll

        finalPose = new Pose(x, y, heading);
    }

    private void predict() {

        x += odoVelocity.getXVelocity() * dt;
        y += odoVelocity.getYVelocity() * dt;
        heading = MathUtil.normalizeAngleRad(heading + (odoVelocity.getAngularVelocity() * dt));

        covariance.replace("Pxx", covariance.get("Pxx") + processNoise.get("Qx"));
        covariance.replace("Pyy", covariance.get("Pyy") + processNoise.get("Qy"));
        covariance.replace("Phh", covariance.get("Phh") + processNoise.get("Qh"));
    }

    private void updateState(double zx, double zy, double zh, double r) {

        double xInnovation = zx - x;
        double yInnovation = zy - y;
        double hInnovation = MathUtil.normalizeAngleRad(zh - heading);

        // Innovation covariance (S = P + R)
        double Sx = covariance.get("Pxx") + r;
        double Sy = covariance.get("Pyy") + r;
        double Sh = covariance.get("Phh") + r;

        //Kalman gain
        double Kx = covariance.get("Pxx") / Sx;
        double Ky = covariance.get("Pyy") / Sy;
        double Kh = covariance.get("Phh") / Sh;

        x += Kx * xInnovation;
        y += Ky * yInnovation;
        heading = MathUtil.normalizeAngleRad(heading + (Kh * hInnovation));

        covariance.put("Pxx", (1 - Kx) * covariance.get("Pxx"));
        covariance.put("Pyy", (1 - Ky) * covariance.get("Pyy"));
        covariance.put("Phh", (1 - Kh) * covariance.get("Phh"));
    }

    public Pose getOdoPose() {
        return odoPose;
    }

    public Pose getLLPose() {
        return llPose;
    }

    public Pose getFinalPose() {
        return finalPose;
    }
}