package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
@Autonomous (name = "V2AutonLONG(RED)", group = "AAAA_MatchPurpose",  preselectTeleOp = "V2TeleOp_RED")
public class V2AutonLONG_RED extends AutonomousBaseOpMode {

    public class RobotElements {

        public InstantAction setFlywheelToFarSideVelocity() {

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
                //GO TO FIRST INTAKE(RED)

                new ParallelAction(
                        robot.intake(),

                        //GO TO FIRST INTAKE(RED)
                        drive.actionBuilder(initialPose)
                                .splineTo(new Vector2d(33, -43), -Math.PI / 2,
                                        new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))

                        //GO TO SMALL TRIANGLE
                                //.splineToSplineHeading(new Pose2d(22, 4, -180), Math.toRadians(-90))

                                .setReversed(true)

                                .splineToSplineHeading(new Pose2d(25, -21, Math.toRadians(-145)), -Math.PI / 2,
                                        new TranslationalVelConstraint(95), new ProfileAccelConstraint(-50, 50))
                                .splineToSplineHeading(new Pose2d(22, 4, -Math.PI), -Math.PI / 2,
                                        new TranslationalVelConstraint(60), new ProfileAccelConstraint(-33, 33))

                                .waitSeconds(5)



                                //GO TO SECOND INTAKE(RED)
                                //52, -44
                                .splineToSplineHeading(new Pose2d(38, -30, Math.toRadians(135)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(52, -44, Math.toRadians(45)), Math.toRadians(45))


                                //GO TO SMALL TRIANGLE
                                .splineToSplineHeading(new Pose2d(30, -16, Math.toRadians(-135)), Math.PI / 2,
                                        new TranslationalVelConstraint(95), new ProfileAccelConstraint(-50, 50))
                                .splineToSplineHeading(new Pose2d(22, -16, -Math.PI), -Math.PI / 2,
                                        new TranslationalVelConstraint(60), new ProfileAccelConstraint(-33, 33))

                                .waitSeconds(5)











                                .build()

                );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(mainPath));
        //SHOOT!
        telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
        telemetry.update();
    }

}
