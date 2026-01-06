package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "turret tuner")
@Config
public class TurretTuner extends OpMode{
    public static double kp, ki, kd;

    Turret turret;

    Pose2d initialPose;
   Telemetry telemetry;


    @Override
    public void init() {

        initialPose = new Pose2d(0, 0, 0);


        turret = new Turret(hardwareMap, initialPose, 72, 72);

    }




    @Override
    public void loop() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());


        turret.setPIDFCoefficients(kp, ki, kd);

        turret.update();

            telemetry.addData("p: ", turret.p);
            telemetry.addData("i: ", turret.i);
            telemetry.addData("d: ", turret.d);
            telemetry.addData("Turret Target: ", turret.turretTargetTicks);
            telemetry.addData("Turret Current Pos: ", turret.turretCurrPosTicks);


        }
}



