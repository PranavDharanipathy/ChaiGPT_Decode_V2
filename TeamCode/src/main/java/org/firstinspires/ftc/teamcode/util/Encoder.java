package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private DcMotorEx motor;

    private Direction direction;
    public Encoder(DcMotorEx motor) {

        this.motor = motor;

        this.direction = Direction.FORWARD;
    }

    public Direction getDirection() {
        return direction;
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        int multiplier = direction.getMultiplier();
        return motor.getCurrentPosition() * multiplier;
    }

    public double getRawVelocityFromInternal() {
        int multiplier = direction.getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    private double filteredVelocity = 0;

    private double kalmanGain = 0;

    private double p = 1; //prediction step
    private double q = 0; //uncertainty step
    private double r = 0; //encoder uncertainty

    private double outlierSigma = 1;
    private double kRInflation = 1; //when outlier is caught

    private boolean velocityKFInitialized = false;
    public void initializeVelocityKalmanFilter(double q, double r, double outlierSigma, double kRInflation) {

        velocityKFInitialized = true;

        if (this.q != q || this.r != r) {
            p = 1;
        }

        this.q = q;
        this.r = r;

        this.outlierSigma = outlierSigma;
        this.kRInflation = kRInflation;
    }

    /// Uses Kalman Filter.
    /// @param velocityEstimate (curr_ticks - prev_ticks) / dt
    public void runVelocityCalculation(double velocityEstimate/*, Telemetry telemetry*/) {

        if (!velocityKFInitialized) throw new RuntimeException("Velocity Kalman Filter is not being used!");

        double innovation = velocityEstimate - filteredVelocity;

        p += q;
        //telemetry.addData("p", p);

        double jump = p + r;

        double correctedR = r; //deals with outliers
        if (Math.abs(innovation) > outlierSigma * Math.sqrt(jump)) {
            correctedR = kRInflation * r;
        }

        //telemetry.addData("correctedR", correctedR);

        double kalmanGainDenominator = p + correctedR;
        kalmanGain = kalmanGainDenominator != 0 ? p / kalmanGainDenominator : 0;
        //telemetry.addData("kalmanGain", kalmanGain);

        filteredVelocity += kalmanGain * innovation;
        //telemetry.addData("filteredVelocity", filteredVelocity);

        p *= (1 - kalmanGain);
    }

    /// In ticks per second.
    /// Using Kalman Filter.
    /// @return The filtered velocity -- will return 0 if Kalman Filter wasn't initialized.
    public double getRealVelocity() {
        return filteredVelocity;
    }
}