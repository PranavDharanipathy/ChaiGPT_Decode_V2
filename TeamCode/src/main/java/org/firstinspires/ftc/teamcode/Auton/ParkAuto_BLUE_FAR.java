package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "ParkAuto BLUE FAR", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp BLUE")
public class ParkAuto_BLUE_FAR extends AutonomousBaseOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0,0,0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        fullInit();

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(-25, -20), Math.toRadians(0))
                        .build()
        );
    }
}
