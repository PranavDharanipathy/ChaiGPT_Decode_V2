package org.firstinspires.ftc.teamcode.Auton;

import static android.os.Build.VERSION_CODES.M;
import static android.renderscript.Sampler.Value.LINEAR;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot.zyxOrientation;
import static com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerYawAxis.Z;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Vector;


@Config

@Autonomous(name = "Nikhil Test Auton Paths")
public class NikhilTestAutonPaths extends AutonomousBaseOpMode {



    public static double DISTANCE = 0;

    public static double ANGLE = 90;



        public NikhilTestAutonPaths(PATHNAME userChoice) {
        this.userChoice = userChoice;
            }


    public enum PATHNAME {
        LINEAR,
        SPLINE,
        CONSTANT
    }

    final PATHNAME userChoice;




    @Override
    public void runOpMode() throws InterruptedException {
        fullInit();

        //userChoice = new PATHNAME();


        System.out.println("WELCOME TO THE AUTON PATH VISUALIZER!");
        System.out.println("To get started, use the key below:");
        System.out.println("WHAT TO TYPE   |  WHAT WILL HAPPEN ");
        System.out.println("   spline      |  SplineToSplineHeading ");
        System.out.println("   linear      |  SplineToLinearHeading ");
        System.out.println("   constant    |  splineToConstantHEading ");

        System.out.println("NOTE: ANGLES MUST BE NEGATIVE");

        telemetry.speak("WELCOME TO THE AUTON PATH VISUALIZER!");
        telemetry.speak("This was created to help you better visualize auton paths(obviously🤣)");

        telemetry.addData("", "WELCOME TO THE AUTON PATH VISUALIZER!");
        telemetry.addData("", "To get started, use the key below:");
        telemetry.addData("", "WHAT TO TYPE   |  WHAT WILL HAPPEN ");
        telemetry.addData("", "   spline      |  SplineToSplineHeading ");
        telemetry.addData("", "   linear      |  SplineToLinearHeading ");
        telemetry.addData("", "   constant    |  splineToConstantHEading ");

        telemetry.addData("", "NOTE: ANGLES MUST BE NEGATIVE");


        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        Action splineToLinear =
                drive.actionBuilder(initialPose)
                        .splineToLinearHeading(new Pose2d(DISTANCE, 20, Math.toRadians(ANGLE)), 0)

                        .build();


        Action splineToSpline =
                drive.actionBuilder(initialPose)
                        .splineToSplineHeading(new Pose2d(DISTANCE, 20, Math.toRadians(ANGLE)), 0)

                        .build();


        Action splineToConstant =
                drive.actionBuilder(initialPose)
                        .splineToConstantHeading(new Vector2d(DISTANCE, 20), 0)

                        .build();



            if (userChoice == PATHNAME.LINEAR) {
                Actions.runBlocking(splineToLinear);
            }
            else if (userChoice == PATHNAME.SPLINE) {
                Actions.runBlocking(splineToSpline);

            }
            else if (userChoice == PATHNAME.CONSTANT) {
                Actions.runBlocking(splineToConstant);
            }




        else {
            System.out.println("PATH NAME NOT FOUND. PLEASE USE A NAME FROM THE KEY ABOVE.");
            telemetry.speak("PATH NAME NOT FOUND. PLEASE USE A NAME FROM THE KEY ABOVE.");
            telemetry.addData("", "PATH NAME NOT FOUND. PLEASE USE A NAME FROM THE KEY ABOVE.");
        }






    }
}
