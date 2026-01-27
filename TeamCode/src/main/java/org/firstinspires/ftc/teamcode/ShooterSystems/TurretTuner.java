package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Turret PID Tuner")
@Config
public class TurretTuner extends LinearOpMode{
    public static double kp, ki, kd;

   private Telemetry telemetry;


   Pose2d initialPose;


   Turret turret;

   int totalTime = 0;






    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        initialPose = new Pose2d(0, 0, 0);


        turret = new Turret(hardwareMap, initialPose, 72, 72);

        totalTime = totalTime + 1;

        waitForStart();

        while (opModeIsActive()) {

            turret.setPIDFCoefficients(kp, ki, kd);

            turret.update();

            turret.updatePID();

            telemetry.addData("p: ", turret.p);
            telemetry.addData("i: ", turret.i);
            telemetry.addData("d: ", turret.d);
            telemetry.addData("Turret Current Pos: ", turret.turretCurrPosTicks);

            telemetry.addData("currX", turret.currX);
            telemetry.addData("currY", turret.currY);
            telemetry.addData("dX", turret.dX);
            telemetry.addData("dY", turret.dY);

            telemetry.addData("turn ticks", turret.turnticks);

            telemetry.addData("bot pose", turret.currentPose);


            telemetry.update();

        }

    }
    }



