package org.firstinspires.ftc.teamcode.pedroPathing;

public class PoseVelocity {

    private double xVelocity;
    private double yVelocity;
    private double angVelocity;
    public PoseVelocity(double xVelocity, double yVelocity, double angVelocity) {

        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.angVelocity = angVelocity;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public double getAngularVelocity() {
        return angVelocity;
    }
}
