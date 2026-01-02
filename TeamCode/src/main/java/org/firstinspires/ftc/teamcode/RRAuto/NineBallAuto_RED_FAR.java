package org.firstinspires.ftc.teamcode.RRAuto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;


import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


import org.firstinspires.ftc.teamcode.Constants;

//@Config
@Autonomous (name = "NineBallAuto RED FAR", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp_RED")
public class NineBallAuto_RED_FAR extends AutonomousBaseOpMode {


    public static double[] TURRET_POSITIONS = {1200, -2100, 1120};


    public class RobotElements {

        public class AllUpdate implements Action {


            private ElapsedTime timer = new ElapsedTime();


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if (timer.milliseconds() > Constants.FLYWHEEL_PIDFVAS_LOOP_TIME) {


                    flywheel.update();
                    timer.reset();
                }


                turret.update();

                telemetry.addData("turret target position", turret.getTargetPosition());
                telemetry.addData("turret current position", turret.getCurrentPosition());

                telemetry.addData("flywheel target velocity", flywheel.getTargetVelocity());
                telemetry.addData("flywheel current velocity", flywheel.getRealVelocity());
                telemetry.update();

                if (opModeIsActive()) {
                    return true;
                }
                else {
                    flywheel.setVelocity(0, true);
                    return false;
                }
            }
        }


        public Action updates() {
            return new AllUpdate();
        }
        public InstantAction setFlywheelToFarSideVelocity() {

            return new InstantAction(() -> flywheel.setVelocity(380_000, true));
        }
        public InstantAction stopFlywheel() {
            return new InstantAction(() -> flywheel.setVelocity(0, true));
        }


        //transfer
        public InstantAction antiTransfer() {
            return new InstantAction(() -> transfer.setVelocity(-300));
        }
        public InstantAction transferArtifact() {
            return new InstantAction(() -> transfer.setVelocity(2000));
        }
        //intake
        public InstantAction reverseIntake() {
            return new InstantAction(() -> intake.setPower(Constants.REVERSE_INTAKE_POWER));
        }


        public InstantAction intake() {
            return new InstantAction(() -> intake.setPower(1));
        }

        public class WaitTilFlywheelAtVelocity implements Action {

            private ElapsedTime timer = new ElapsedTime();

            private double minimumTime;
            private double minimumVelocity;

            public WaitTilFlywheelAtVelocity(double minimumTime, double minimumVelocity) {

                this.minimumTime = minimumTime;
                this.minimumVelocity = minimumVelocity;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                return !(timer.seconds() >= minimumTime && flywheel.getRealVelocity() >= minimumVelocity);
            }
        }

        public Action waitTilFlywheelAtVelocity(double minimumTime, double minimumVelocity) {
            return new WaitTilFlywheelAtVelocity(minimumTime, minimumVelocity);
        }

        public Action firstShootSequence() {

            return new SequentialAction(

                    new SleepAction(3.5),
                    waitTilFlywheelAtVelocity(1, 379_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(1.85, 379_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(1.85, 379_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    new InstantAction(() -> flywheel.setVelocity(420_000, false)),
                    antiTransfer(),

                    //setup for second
                    new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[1]))
            );
        }

        public Action secondShootSequence() {

            return new SequentialAction(

                    waitTilFlywheelAtVelocity(0.25, 419_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2, 419_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2, 419_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    //setup for third
                    new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2]))
            );
        }

        public Action thirdShootSequence() {

            return new SequentialAction(

                    waitTilFlywheelAtVelocity(0.25, 419_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2, 419_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2, 419_000),
                    transferArtifact(),
                    new SleepAction(0.4),
                    new InstantAction(() -> transfer.setVelocity(0))
            );
        }

    }
    public double turretStartPosition;


    @Override
    public void runOpMode() throws InterruptedException {


        fullInit();


        final RobotElements robot = new RobotElements();


        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Action mainPath =




                //MOVE TO FIRST INTAKE POINT


                new ParallelAction(


                        robot.intake(),
                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),

                        drive.actionBuilder(initialPose)


                                //preload
                                .splineToLinearHeading(new Pose2d(-6, 6, Math.toRadians(-36)), 0)

                                .stopAndAdd(robot.firstShootSequence())

                                //first intake

                                .splineToLinearHeading(new Pose2d(-33, 7.5, Math.PI / 2), Math.toRadians(-36))
                                .waitSeconds(0.1)
                                .splineToConstantHeading(new Vector2d(-33, 45), Math.PI / 2)

                                //GO TO SMALL TRIANGLE
                                .setReversed(true)


                                .splineToSplineHeading(new Pose2d(-13, 7, 0), Math.PI / 2)


                                .stopAndAdd(robot.secondShootSequence())


                                .setReversed(false)


                                ///.splineToSplineHeading(new Pose2d(-44, 0, -Math.PI / 2), 0,
                                //new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))


                                //SECOND INTAKE


                                .splineToLinearHeading(new Pose2d(-56, 4, Math.PI / 2), 0)
                                .waitSeconds(0.1)
                                .splineToConstantHeading(new Vector2d(-56, 35), -Math.PI / 2)


                                //GO TO SMALL TRIANGLE


                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-38, 30), Math.PI / 2)
                                .splineToSplineHeading(new Pose2d(-7, 7, Math.toRadians(-36)), Math.PI / 2)

                                .stopAndAdd(
                                        new SequentialAction(

                                                robot.thirdShootSequence(),
                                                new InstantAction(() -> turret.setPosition(turretStartPosition)),
                                                robot.stopFlywheel()
                                        )
                                )

                                //movement rp
                                .splineToLinearHeading(new Pose2d(-20, 12, Math.toRadians(0)), Math.toRadians(-36))
                                .build());






        //using starting position as home position
        turretStartPosition = turret.getCurrentPosition();

        waitForStart();


        if (isStopRequested()) return;






        Actions.runBlocking(
                new ParallelAction(

                        new InstantAction(() -> hoodAngler.setPosition(ShooterInformation.ShooterConstants.HOOD_FAR_POSITION)),

                        robot.setFlywheelToFarSideVelocity(),
                        robot.updates(),


                        robot.antiTransfer(),
                        mainPath
                )
        );

    }
}