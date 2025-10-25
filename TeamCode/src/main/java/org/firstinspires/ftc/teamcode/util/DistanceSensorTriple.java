package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorTriple {

    private DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH; //in by default

    public void setDistanceUnit(DistanceUnit DISTANCE_UNIT) { this.DISTANCE_UNIT = DISTANCE_UNIT; }

    public DistanceUnit getDistanceUnit() { return DISTANCE_UNIT; }



    private Rev2mDistanceSensor left, back, right;

    public DistanceSensorTriple(HardwareMap hardwareMap, String leftDistanceSensorName, String backDistanceSensorName, String rightDistanceSensorName) {

        left = hardwareMap.get(Rev2mDistanceSensor.class, leftDistanceSensorName);
        back = hardwareMap.get(Rev2mDistanceSensor.class, backDistanceSensorName);
        right = hardwareMap.get(Rev2mDistanceSensor.class, rightDistanceSensorName);
    }

    /// @return 3 distances in the order of left, back, and right. If it returns null, that means that the distance is invalid.
    public Double[] getDistances() {

        double leftDistance = left.getDistance(DISTANCE_UNIT);
        double backDistance = back.getDistance(DISTANCE_UNIT);
        double rightDistance = right.getDistance(DISTANCE_UNIT);

        return new Double[] {
                isDistanceInvalid(leftDistance) ? null : leftDistance,
                isDistanceInvalid(backDistance) ? null : backDistance,
                isDistanceInvalid(rightDistance) ? null : rightDistance
        };
    }

    ///  @return true means invalid, false means valid
    private boolean isDistanceInvalid(double distance) {

        boolean isInvalid = false;

        if (distance > 320) isInvalid = true;
        else if (distance <= 0.1) isInvalid = true;

        return isInvalid;
    }
}
