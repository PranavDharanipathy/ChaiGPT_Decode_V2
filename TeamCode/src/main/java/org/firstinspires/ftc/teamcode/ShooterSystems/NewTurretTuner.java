package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class NewTurretTuner extends OpMode {

    NewTurret turret;

    Pose initialPose;
    public  static double kp, ki, kd;




    @Override
    public void init() {
        initialPose = new Pose(72, 72, 0);
        turret = new NewTurret(hardwareMap, initialPose, 25, 100);
    }

    @Override
    public void loop() {

        turret.setPID(kp, ki, kd);

        turret.update();

    }
}
