package org.firstinspires.ftc.teamcode.ShooterSystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretHelper {

    ///converts angle into position
    public static double fromAngle(double angle, double turretStartPosition) {
        return angle * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;
    }

    ///converts position into angle
    public static double toAngle(double position, double turretStartPosition) {
        return (position - turretStartPosition) / ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    private static double dt;

    private static double prevVelocity, currVelocity, acceleration;

    /// REQUIRES {@link TurretHelper#update} to be run.
    public static double getVelocity() {
        return currVelocity;
    }

    /// REQUIRES {@link TurretHelper#update} to be run.
    public static double getVelocity(AngleUnit angleUnit) {

        final double degVel = currVelocity / ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE;

        return angleUnit == AngleUnit.DEGREES ? degVel : Math.toRadians(degVel);
    }

    /// REQUIRES {@link TurretHelper#update} to be run.
    public static double getAcceleration() {
        return acceleration;
    }

    /// REQUIRES {@link TurretHelper#update} to be run.
    public static double getAcceleration(AngleUnit angleUnit) {

        final double degAccel = acceleration / ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE;

        return angleUnit == AngleUnit.DEGREES ? degAccel : Math.toRadians(degAccel);
    }

    private static double getSeconds() {
        return System.nanoTime() * 1e-9;
    }

    private static double prevTime = 0, currTime = 0;

    public static void update(TurretBase turretBase) {

        prevTime = currTime;
        currTime = getSeconds();

        dt = currTime - prevTime;

        prevVelocity = currVelocity;
        currVelocity = (turretBase.getCurrentPosition() - turretBase.getLastCurrentPosition()) / dt;

        acceleration = (currVelocity - prevVelocity) / dt;
    }
}
