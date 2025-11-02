package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name = "V2Auton")
public abstract class V2Auton extends OpMode {

    public InstantAction intake() {
        return new InstantAction(() -> intake.setVelocity(Constants.BASE_INTAKE_VELOCITY));
    }

    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

    CommandScheduler scheduler = new CommandScheduler();


    public void runOpMode() {

        int startPosition = 0;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action firstintake=
                new SequentialAction(
                        drive.actionBuilder(initialPose)
                                .splineToLinearHeading(new Pose2d(42, 54, Math.toRadians(-90)), Math.toRadians(0))
                                .build()
                );

        Action firstshoot =
                new SequentialAction(
                        drive.actionBuilder(initialPose)
                                .strafeTo(new Pose2d(37,50, Math.toRadians(45)),
                                        .build()
                );



    }



}
