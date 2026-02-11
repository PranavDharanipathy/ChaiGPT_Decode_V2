package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.Rev9AxisImuWrapped;

import java.util.ArrayList;
import java.util.List;

public class PoseVelocityTracker {

    private Follower follower;

    public PoseVelocityTracker(Follower follower) {
        this.follower = follower;
    }

    //index 0 is previous and index 1 is current
    private List<Double> xVelHistory = new ArrayList<>(List.of(0.0, 0.0));
    private List<Double> yVelHistory = new ArrayList<>(List.of(0.0, 0.0));
    private List<Double> angVelHistory = new ArrayList<>(List.of(0.0, 0.0));

    private void buildVelHistory(List<Double> velHistory, double currentVel) {

        velHistory.set(0, velHistory.get(1));
        velHistory.set(1, currentVel);
    }

    private double calcVelUnitless(List<Double> velHistory) {

        double[] history = velHistory.stream().mapToDouble(Double::doubleValue).toArray();

        return history[1] - history[0];
    }

    private double xVelocity;
    private double yVelocity;

    private double angularVelocity;

    private double getSeconds() {
        return System.nanoTime() * 1e-9;
    }
    private double prevTime, currTime;

    public void update() {

        prevTime = currTime;
        currTime = getSeconds();

        double dt = currTime - prevTime;

        Pose pose = follower.getPose();

        buildVelHistory(xVelHistory, pose.getX());
        buildVelHistory(yVelHistory, pose.getY());
        buildVelHistory(angVelHistory, pose.getHeading());

        xVelocity = calcVelUnitless(xVelHistory) / dt;
        yVelocity = calcVelUnitless(yVelHistory) / dt;
        angularVelocity = calcVelUnitless(angVelHistory) / dt;
    }

    /// <p>x: in/sec </p>
    /// <p>y: in/sec </p>
    /// <p>heading: rad/sec </p>
    public PoseVelocity getPoseVelocity() {
        return new PoseVelocity(xVelocity, yVelocity, angularVelocity);
    }

    /// Index 0 is xVelHistory, index 1 is yVelHistory, index 2 is angVelHistory
    public double[][] getHistories() {

        double[] xH = xVelHistory.stream().mapToDouble(Double::doubleValue).toArray();
        double[] yH = yVelHistory.stream().mapToDouble(Double::doubleValue).toArray();
        double[] angH = angVelHistory.stream().mapToDouble(Double::doubleValue).toArray();

        return new double[][] {xH, yH, angH};
    }
}
