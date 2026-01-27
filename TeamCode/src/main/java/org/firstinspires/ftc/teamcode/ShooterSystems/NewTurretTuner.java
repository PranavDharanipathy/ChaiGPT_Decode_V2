package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "NEW PID TUNER")
public class NewTurretTuner extends OpMode {

    NewTurret turret;

    Pose initialPose;
    public  static double kp, ki, kd;

    Telemetry telemetry;




    @Override
    public void init() {
        initialPose = new Pose(72, 72, 0);
        turret = new NewTurret(hardwareMap, initialPose, 25, 100);
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        turret.setPID(kp, ki, kd);

        turret.update();

        telemetry.addData("robotX: ", turret.currX);

        telemetry.addData("robotY: ", turret.currY);

        telemetry.addData("robotHeading: ", turret.robotHeading);

        telemetry.addData("turretTurn ", turret.turretTurnTicks);

        telemetry.update();



    }
}
