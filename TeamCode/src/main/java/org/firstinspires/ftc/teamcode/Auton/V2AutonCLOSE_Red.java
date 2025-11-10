package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.TeleOp.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class V2AutonCLOSE_Red extends AutonomousBaseOpMode {

    public class RobotElements {
        public InstantAction setFlywheelToFarSideVelocity() {
            Intake intake = new Intake();
            return new InstantAction(() -> flywheel.setVelocity(ShooterInformation.ShooterConstants.FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY, true));
        }
        public InstantAction stopFlywheel() {
            return new InstantAction(() -> flywheel.setVelocity(0, true));
        }

        //transfer
        public InstantAction antiTransfer() {
            return new InstantAction(() -> transfer.setVelocity(Constants.ANTI_TRANSFER_VELOCITY));
        }
        public InstantAction transferArtifact() {
            return new InstantAction(() -> transfer.setVelocity(Constants.TRANSFER_VELOCITY));
        }
        //intake
        public InstantAction reverseIntake() {
            return new InstantAction(() -> intake.setVelocity(Constants.REVERSE_INTAKE_VELOCITY));
        }

        public InstantAction intake() {
            return new InstantAction(() -> intake.setVelocity(Constants.BASE_INTAKE_VELOCITY));
        }

    }

    public double turretStartPosition;

    @Override
    public void runOpMode() throws InterruptedException {


        fullInit();

        final RobotElements robot = new RobotElements();

        turretStartPosition = turret.getCurrentPosition();
        telemetry.addData("turret current position", turretStartPosition);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        int startPosition = 0;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action mainPath =

                new ParallelAction(
                        drive.actionBuilder(initialPose)
                                //PRELOAD(big triangle, red zone)
                                .splineToSplineHeading(new Pose2d(36, 54, 90), 45)

                                //FIRST INTAKE

                                .splineToSplineHeading(new Pose2d(-45, -1, 180), Math.toRadians(90))

                                //big triangle(red zone)

                                .setReversed(true)

                                .splineToSplineHeading(new Pose2d(36, 54, 90), 180)


                                //SECOND INTAKE

                                .setReversed(false)

                                .splineTo(new Vector2d(-70, -2), Math.toRadians(90))

                                .setReversed((true))

                                .splineToSplineHeading(new Pose2d(36, 54, 90), 180)






                                        .build()
                );




    }
}
