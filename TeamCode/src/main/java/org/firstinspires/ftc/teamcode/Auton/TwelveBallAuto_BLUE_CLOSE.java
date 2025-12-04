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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.NewMecanumDrive;


@Config
@Autonomous (name = "12 Ball BLUE CLOSE(AUTO)", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp_BLUE")
public class TwelveBallAuto_BLUE_CLOSE extends AutonomousBaseOpMode {
    public static double[] TURRET_POSITIONS = {5900, 6300 , 5800, 6100};

    public static double[] HOOD_ANGLING = {0.25, 0.27, 0};
    public class RobotElements {
        public class AllUpdate implements Action {
            private ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (timer.milliseconds() > Constants.FLYWHEEL_PIDFVAS_LOOP_TIME) {
                    flywheel.updateKvBasedOnVoltage();
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

        public class FlywheelWaitVel implements Action {
            private ElapsedTime timer = new ElapsedTime();

            private double minimumTime;

            public FlywheelWaitVel(double minimumTime) {this.minimumTime = minimumTime;}

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                telemetry.update();
                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() > 22500 && flywheel.getLastFrontendCalculatedVelocity() > 22500);
            }


        }


        public Action updates() {
            return new RobotElements.AllUpdate();
        }

        public InstantAction setFlywheelToCloseSideVelocity() {

            return new InstantAction(() -> flywheel.setVelocity(24600, true));
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

        public InstantAction fact1() {
            return new InstantAction(() -> telemetry.speak("Did you know that A crocodile cannot stick its tongue out?"));
        }
        public Action waitFlywheelVel(double minimumTime) {
            return new FlywheelWaitVel(minimumTime);
        }

        public double turretStartPosition;
        public Action firstShootSequence() {

            return new SequentialAction(
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),
                    new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLING[0])),

                    waitFlywheelVel(5),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(4),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer()

                    //setup for second
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
            );
        }

        public Action secondShootSequence() {

            return new SequentialAction(

                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer()

                    //setup for third
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
            );
        }

        public Action thirdShootSequence() {

            return new SequentialAction(
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.5),
                    antiTransfer()
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
            );
        }

        public Action fourthShootSequence() {
            return new SequentialAction(
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer()
            );
        }

    }


    public double turretStartPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        fullInit();

        final RobotElements robot = new RobotElements();
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action mainPath =
                new ParallelAction(
                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),
                        robot.intake(),
                        //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),

                        //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[1])),
                        drive.actionBuilder(initialPose)
                                //PRELOAD(BIG TRIANGLE)
                                //original x = -30, Tested using y = 0.

                                .splineToLinearHeading(new Pose2d(-38, -26, Math.toRadians(145)), Math.toRadians(225))
                                //SHOOT
                                //.stopAndAdd(robot.fact1())

                                .stopAndAdd(
                                        new SequentialAction(
                                                new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),

                                                robot.firstShootSequence()
                                                //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))

                                        )
                                )

                                //FIRST INTAKE

                                .splineToLinearHeading(new Pose2d(-57, 6, Math.toRadians(90)), Math.toRadians(145))

                                .lineToY(-2)

                                //.splineToLinearHeading(new Pose2d(-57, 2, Math.toRadians(90)), Math.toRadians(90))

                                //.splineToConstantHeading(new Vector2d(-53, 10), Math.toRadians(90))

                                //OPEN GATE
                                .splineToConstantHeading(new Vector2d(-49, -6), Math.toRadians(90))

                                .splineToLinearHeading(new Pose2d(-49, 9, Math.toRadians(-90)), Math.PI / 2)

                               // .splineToConstantHeading(new Vector2d(-61, 7), -Math.PI / 2)
                                .waitSeconds(2.3)

                                //.waitSeconds(3)

                                //GO TO BIG TRIANGLE

                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-41, -27, Math.toRadians(145)), Math.toRadians(90))

                                .stopAndAdd(
                                        new SequentialAction(
                                                new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[3])),
                                                robot.secondShootSequence()
                                                //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
                                        )
                                )

                                 //SECOND INTAKE

                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-79, -15, Math.PI / 2), Math.toRadians(145))

                                .splineToLinearHeading(new Pose2d(-79, 11, Math.PI / 2), Math.toRadians(90))

                                //GO TO BIG TRIANGLE

                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-41, -26, Math.toRadians(145)), Math.toRadians(90))

                                .stopAndAdd(
                                        new SequentialAction(
                                                //new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLING[1])),
                                                new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
                                                robot.thirdShootSequence()
                                                //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
                                        )
                                )



                                //THIRD INTAKE
                                .setReversed(false)

                                .splineToLinearHeading(new Pose2d(-105, -11, Math.PI / 2), Math.toRadians(145))

                                .splineToLinearHeading(new Pose2d(-105, 17, Math.PI / 2), Math.toRadians(90))


                                //11/28/2025 TEST CODE

                                //GO TO SMALL TRIANGLE

                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-41, -27, Math.toRadians(145)), Math.toRadians(90))

                                .stopAndAdd(
                                        new SequentialAction(
                                                new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[1])),
                                                robot.fourthShootSequence()
                                                //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
                                        )
                                )


                                .build()
                );

        turretStartPosition = turret.getCurrentPosition();
        waitForStart();


        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(

                        new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLING[0])),

                        //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[1])),


                        robot.setFlywheelToCloseSideVelocity(),
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
