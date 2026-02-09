package org.firstinspires.ftc.teamcode.data;

import com.pedropathing.geometry.Pose;

public class LocalizationData {

    private Pose pose;
    private Double turretStartPosition;

    public LocalizationData(Pose pose, Double turretStartPosition) {

        this.pose = pose;
        this.turretStartPosition = turretStartPosition;
    }

    public Pose getPose() {
        return pose;
    }

    public Double getTurretStartPosition() {
        return turretStartPosition;
    }

}
