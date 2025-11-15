package org.firstinspires.ftc.teamcode.Auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous (name = "NineBallAuto BLUE FAR", group = "AAAA_MatchPurpose", preselectTeleOp = "V2Teleop_BLUE")
public class NineBallAuto_BLUE_FAR extends AutonomousBaseOpMode {


    public static double[] TURRET_POSITIONS = {-1000, 2100, -1100};


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

            return new InstantAction(() -> flywheel.setVelocity(36_000, true));
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


        public class WaitTilFlywheelAtVelocity implements Action {

            private ElapsedTime timer = new ElapsedTime();

            private double minimumTime;

            public WaitTilFlywheelAtVelocity(double minimumTime) {
                this.minimumTime = minimumTime;
            }


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                telemetry.update();
                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() > 30500 && flywheel.getLastFrontendCalculatedVelocity() > 30500);
            }
        }

        public Action waitTilFlywheelAtVelocity(double minimumTime) {
            return new WaitTilFlywheelAtVelocity(minimumTime);
        }

        public Action firstShootSequence() {

            return new SequentialAction(
                    waitTilFlywheelAtVelocity(4),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    //setup for second
                    new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[1]))
            );
        }

        public Action secondShootSequence() {

            return new SequentialAction(
                    waitTilFlywheelAtVelocity(4),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(3),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(3),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    //setup for third
            new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2]))
            );
        }

        public Action thirdShootSequence() {

            return new SequentialAction(
                    waitTilFlywheelAtVelocity(4),
                    transferArtifact(),
                    new SleepAction(0.3),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2),
                    transferArtifact(),
                    new SleepAction(0.3),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2),
                    transferArtifact(),
                    new SleepAction(0.3),
                    antiTransfer()
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
                                .splineToLinearHeading(new Pose2d(-18, -7.5, Math.toRadians(36)), 0)

                                .stopAndAdd(robot.firstShootSequence())

                                //first intake
                                .splineTo(new Vector2d(-21, -57), -Math.PI / 2)




                                //GO TO SMALL TRIANGLE
                                .setReversed(true)


                                .splineToSplineHeading(new Pose2d(-15, -7, 0), -Math.PI / 2)


                                .stopAndAdd(robot.secondShootSequence())


                                .setReversed(false)


                                ///.splineToSplineHeading(new Pose2d(-44, 0, -Math.PI / 2), 0,
                                //new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))


                                //SECOND INTAKE


                                .splineToSplineHeading(new Pose2d(-44, -19, -Math.PI / 2), 0)
                                .waitSeconds(0.1)
                                .splineToConstantHeading(new Vector2d(-44, -59), -Math.PI / 2)


                                //GO TO SMALL TRIANGLE


                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-38, -30), -Math.PI / 2)
                                .splineToSplineHeading(new Pose2d(-18, -6, Math.toRadians(36)), -Math.PI / 2)

                                .stopAndAdd(
                                        new SequentialAction(

                                                robot.thirdShootSequence(),
                                                new InstantAction(() -> turret.setPosition(turretStartPosition))
                                        )
                                )

                                //movement rp
                                .splineToLinearHeading(new Pose2d(-20, -12, Math.toRadians(0)), Math.toRadians(36))
                                .build());






        turretStartPosition = turret.getCurrentPosition();
        telemetry.addData("turret current position", turretStartPosition);
        telemetry.update();


        waitForStart();


        if (isStopRequested()) return;






        Actions.runBlocking(
                new ParallelAction(

                        new InstantAction(() -> hoodAngler.setPosition(0.115)),

                        robot.setFlywheelToFarSideVelocity(),
                        robot.updates(),


                        robot.antiTransfer(),
                        mainPath
                )
        );
        //SHOOT!
        telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
        telemetry.update();
    }
}
