package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.Rev9AxisImu;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Rev9AxisImuWrapped {

    private Rev9AxisImu rev9AxisImu;

    private double headingHomeDeg = 0;

    public Rev9AxisImuWrapped(Rev9AxisImu rev9AxisImu) {
        this.rev9AxisImu = rev9AxisImu;
    }

    public Rev9AxisImu accessUnwrappedImu() {
        return rev9AxisImu;
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

}
