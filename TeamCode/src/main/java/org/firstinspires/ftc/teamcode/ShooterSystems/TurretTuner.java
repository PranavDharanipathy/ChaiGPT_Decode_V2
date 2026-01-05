package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name = "turret tuner")
public class TurretTuner extends LinearOpMode {
    double kp, ki, kd;

    Telemetry telemetry;

    Pose2d initialPose = new Pose2d(0, 0, 0);


    Turret turret = new Turret(hardwareMap, initialPose, 72, 72);


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());


    turret.setPIDFCoefficients(kp, ki, kd);


    turret.updatePID();

        telemetry.addData("p: ", turret.p);
        telemetry.addData("i: ", turret.i);
        telemetry.addData("d: ", turret.d);
        telemetry.addData("Turret Target: ", turret.turretTargetTicks);
        telemetry.addData("Turret Current Pos: ", turret.turretCurrPosTicks);


    }
}



