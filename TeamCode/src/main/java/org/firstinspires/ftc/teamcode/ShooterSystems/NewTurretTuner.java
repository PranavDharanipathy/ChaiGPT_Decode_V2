package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class NewTurretTuner extends OpMode {

    NewTurret turret;

    Pose2d initialPose;
    public  static double kp, ki, kd;




    @Override
    public void init() {
        initialPose = new Pose2d(0, 0, 0);
        turret = new NewTurret(hardwareMap, initialPose, 50, 50);
    }

    @Override
    public void loop() {

        turret.setPID(kp, ki, kd);

        turret.update();

    }
}
