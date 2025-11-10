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
                new ParallelAction(
                        robot.intake(),

                        drive.actionBuilder(initialPose)
                                //PRELOAD POSITION TODO: Test and make angle more accurate
                                .splineToSplineHeading(new Pose2d(5, 4, Math.toRadians(140)), 0, new TranslationalVelConstraint(90), new ProfileAccelConstraint(-70, 70))

                                //FIRST INTAKE

                                .splineTo(new Vector2d(33, -43), -Math.PI / 2,
                                        new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))

                                //SMALL TRIANGLE
                                .splineToSplineHeading(new Pose2d(13, -13, -186), -90,
                                        new TranslationalVelConstraint(90), new ProfileAccelConstraint(-33, 33))

                                .waitSeconds(5)

                                //SECOND INTAKE
                                //(52, -44)
                                .splineToSplineHeading(new Pose2d(38, -43, Math.toRadians(-90)), Math.toRadians(180),
                                        new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50, 50))

                                //GO TO SMALL TRIANGLE
                                .splineToSplineHeading(new Pose2d(13, -13, -186), -90,
                                        new TranslationalVelConstraint(90), new ProfileAccelConstraint(-33, 33))

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
