package org.firstinspires.ftc.teamcode.data;

import com.pedropathing.geometry.Pose;

public class LocalizationData {

    private Pose pose;
    private Double turretStartPosition;

    public LocalizationData(Pose pose, Double turretStartPosition) {

        this.pose = pose;
        this.turretStartPosition = turretStartPosition;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public Pose getPose() {
        return pose;
    }

    public void setTurretStartPosition(Double turretStartPosition) {
        this.turretStartPosition = turretStartPosition;
    }

    public Double getTurretStartPosition() {
        return turretStartPosition;
    }

}
