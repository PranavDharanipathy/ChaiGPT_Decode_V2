package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.MediaTransferProtocolMonitorService;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.TeleOp.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous (name = "V2AutonLONG(BLUE)", group = "AAAA_MatchPurpose", preselectTeleOp = "V2Teleop_BLUE")
public class V2AutonLONG_Blue extends AutonomousBaseOpMode {


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
                //MOVE TO FIRST INTAKE POINT

                new ParallelAction(

                                robot.intake(),
                                drive.actionBuilder(initialPose)

                                        //TANGENT = 90
                                        //FIRST INTAKE
                                        .splineTo(new Vector2d(21, 42), Math.PI / 2,
                                                new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))


                                        //GO TO SMALL TRIANGLE
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(20, 6.7, Math.toRadians(135)), Math.PI / 2,
                                                new TranslationalVelConstraint(95), new ProfileAccelConstraint(-50, 50))
                                        .splineToSplineHeading(new Pose2d(20, -11, Math.PI), Math.PI / 2,
                                                new TranslationalVelConstraint(60), new ProfileAccelConstraint(-33, 33))
//1.5 secs
                                        .waitSeconds(3)
                                    //MOVE TO 2nd INTAKE POINT

                                    .setReversed(false)

                                    .splineToSplineHeading(new Pose2d(47, 0, Math.PI / 2), Math.PI,
                                            new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))

                                    .splineToConstantHeading(new Vector2d(46, 46.3), Math.PI / 2,
                                            new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
                                    //.splineTo(new Vector2d(44, 47), Math.PI / 2

                                    //GO TO SMALL TRIANGLE
                                    .setReversed(true)
                                    .splineToConstantHeading(new Vector2d(38, 30), Math.PI / 2,
                                                new TranslationalVelConstraint(110), new ProfileAccelConstraint(-75, 75))
                                    .splineToSplineHeading(new Pose2d(20, -12, Math.PI), Math.PI / 2,
                                            new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 60))
//1.5 secs
                                        .waitSeconds(3)

                                    .build());



        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(new ParallelAction(robot.antiTransfer(), mainPath));
        //SHOOT!
        telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
        telemetry.update();
    }
}