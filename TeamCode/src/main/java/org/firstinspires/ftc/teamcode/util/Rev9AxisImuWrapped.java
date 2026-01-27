package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.Rev9AxisImu;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Rev9AxisImuWrapped {

    private Rev9AxisImu rev9AxisImu;

    private double headingHomeDeg = 0;

    public Rev9AxisImuWrapped(Rev9AxisImu rev9AxisImu) {
        this.rev9AxisImu = rev9AxisImu;
    }

    public Rev9AxisImu accessUnwrappedImu() {
        return rev9AxisImu;
    }

    public double getHeadingHomeRad() {
        return Math.toRadians(headingHomeDeg);
    }
    public double getHeadingHomeDeg() {
        return headingHomeDeg;
    }

    public void resetYaw() {

        headingHomeDeg = 0;
        rev9AxisImu.resetYaw();
    }

    public void setYaw(double yawDeg) {
        headingHomeDeg = yawDeg - rev9AxisImu.getRobotYawPitchRollAngles().getYaw();
    }

    public double getYaw() {
        return rev9AxisImu.getRobotYawPitchRollAngles().getYaw() + headingHomeDeg;
    }

    public double getYaw(AngleUnit angleUnit) {
        return rev9AxisImu.getRobotYawPitchRollAngles().getYaw(angleUnit) + (angleUnit == AngleUnit.RADIANS ? Math.toRadians(headingHomeDeg) : headingHomeDeg);
    }

    public double getPitch() {
        return rev9AxisImu.getRobotYawPitchRollAngles().getPitch();
    }

    public double getPitch(AngleUnit angleUnit) {
        return rev9AxisImu.getRobotYawPitchRollAngles().getPitch(angleUnit);
    }

    public double getRoll() {
        return rev9AxisImu.getRobotYawPitchRollAngles().getRoll();
    }

    public double getRoll(AngleUnit angleUnit) {
        return rev9AxisImu.getRobotYawPitchRollAngles().getRoll(angleUnit);
    }

    //index 0 is previous and index 1 is current
    private List<Double> yawVelHistory = new ArrayList<>(List.of(0.0, 0.0));
    private List<Double> pitchVelHistory = new ArrayList<>(List.of(0.0, 0.0));
    private List<Double> rollVelHistory = new ArrayList<>(List.of(0.0, 0.0));

    private double getSeconds() {
        return System.nanoTime() * 1e-9;
    }

    private double yawVel;
    private double pitchVel;
    private double rollVel;

    /// in radians per second
    public double getYawVelocity() {
        return yawVel;
    }

    /// in radians per second
    public double getPitchVelocity() {
        return pitchVel;
    }

    /// in radians per second
    public double getRollVelocity() {
        return rollVel;
    }

    private double prevTime, currTime;

    public void updateVelocities() {

        prevTime = currTime;
        currTime = getSeconds();

        double dt = currTime - prevTime;

        buildVelHistory(yawVelHistory, this::getYaw);
        buildVelHistory(pitchVelHistory, this::getPitch);
        buildVelHistory(rollVelHistory, this::getRoll);

        yawVel = calcVelUnitless(yawVelHistory) * dt;
        pitchVel = calcVelUnitless(pitchVelHistory) * dt;
        rollVel = calcVelUnitless(rollVelHistory) * dt;
    }

    private void buildVelHistory(List<Double> velHistory, DoubleSupplier dataSupplier) {

        velHistory.set(0, velHistory.get(1));
        velHistory.set(1, dataSupplier.getAsDouble());
    }

    private double calcVelUnitless(List<Double> velHistory) {

        double[] history = velHistory.stream().mapToDouble(Double::doubleValue).toArray();

        return history[1] - history[0];
    }
}
