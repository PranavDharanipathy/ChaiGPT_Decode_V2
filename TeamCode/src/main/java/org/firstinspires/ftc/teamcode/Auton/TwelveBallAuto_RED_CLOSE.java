package org.firstinspires.ftc.teamcode.Auton;

import static android.os.Build.VERSION_CODES.R;

import static com.sun.tools.doclint.HtmlTag.P;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.NewMecanumDrive;


@Config
@Autonomous (name = "12 Ball RED CLOSE(AUTO)", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp_RED")
public class TwelveBallAuto_RED_CLOSE extends AutonomousBaseOpMode {
    public static double[] TURRET_POSITIONS = {-3800, -4100 , -4200};

    //PRELOAD = [0]
    //FIRST INTAKE = [2]
    //SECOND INTAKE = [1]

    public static double[] HOOD_ANGLING = {0.18, 0.13, 0};
    //PRELOAD = [0]
    // First intake = [0]
    //SECOND INTAKE = [1]

    public static double FLYWHEEL_VELOCITY = 131500;

    public static double SECONDX = -52;

    public static double SECONDY = -7.8;



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

        public class FlywheelWaitVel implements Action {
            private ElapsedTime timer = new ElapsedTime();

            private double minimumTime;

            public FlywheelWaitVel(double minimumTime) {this.minimumTime = minimumTime;}

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                telemetry.update();
                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() > FLYWHEEL_VELOCITY && flywheel.getLastFrontendCalculatedVelocity() > FLYWHEEL_VELOCITY);
            }


        }


        public Action updates() {
            return new RobotElements.AllUpdate();
        }

        public InstantAction setFlywheelToCloseSideVelocity() {

            return new InstantAction(() -> flywheel.setVelocity(135500, true));
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
            return new InstantAction(() -> intake.setVelocity(2900));
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

                    waitFlywheelVel(1),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer(),

                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer(),

                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.8),
                    antiTransfer(),

                    waitFlywheelVel(5),
                    transferArtifact(),
                    new SleepAction(1.4),
                    antiTransfer()

                    //setup for second
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
            );
        }

        public Action secondShootSequence() {

            return new SequentialAction(

                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer(),

                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer(),

                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.8),
                    antiTransfer(),

                    waitFlywheelVel(5),
                    transferArtifact(),
                    new SleepAction(0.8),
                    antiTransfer()

                    //setup for third
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
            );
        }

        public Action thirdShootSequence() {

            return new SequentialAction(


                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
                    waitFlywheelVel(1),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer(),

                    waitFlywheelVel(5),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer()
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
            );
        }

        public Action fourthShootSequence() {
            return new SequentialAction(
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.3),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.3),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.6),
                    antiTransfer(),

                    waitFlywheelVel(5),
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
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        //DEFINING EXPERIMENTAL FIRST INTAKE TRAJECTORY
        TrajectoryActionBuilder firstIntake = drive.actionBuilder(initialPose)

                .splineToConstantHeading(new Vector2d(-26, 31), Math.PI)

                .splineToLinearHeading(new Pose2d(-28, 21, -Math.PI / 2), Math.PI)

                .splineToConstantHeading(new Vector2d(-28, -2), -Math.PI)

                .endTrajectory();


        Action mainPath =

        new ParallelAction(

                new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),
                robot.intake(),
                drive.actionBuilder(initialPose)
                        //PRELOAD SPLINE

                        .splineToLinearHeading(new Pose2d(-17, 28, Math.toRadians(180)), Math.toRadians(135))

                        .stopAndAdd(
                                new SequentialAction(
                                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
                                        robot.firstShootSequence(),
                                        new InstantAction(() ->turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
                                        //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))

                                )
                        )

                        //FIRST INTAKE

                        .stopAndAdd(firstIntake.build())

                       /* .splineToConstantHeading(new Vector2d(-26, 31), Math.PI)

                        .splineToLinearHeading(new Pose2d(-28, 21, -Math.PI / 2), Math.PI)

                        .splineToConstantHeading(new Vector2d(FIRSTX, FIRSTY), -Math.PI, new TranslationalVelConstraint(50), new ProfileAccelConstraint(-90, 90))
                             */


                        //OPEN GATE

                        .splineToConstantHeading(new Vector2d(-47, 25), Math.toRadians(-90))

                        .splineToLinearHeading(new Pose2d(-47, -3, Math.toRadians(90)), -Math.PI / 2)

                        .waitSeconds(2.5)




                        //GO TO BIG TRIANGLE

                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-17, 28, -Math.PI), Math.toRadians(-90))

                        .stopAndAdd(
                                new SequentialAction(
                                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),
                                        robot.secondShootSequence(),
                                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2]))
                                )
                        )

                        //SECOND INTAKE

                        .setReversed(false)

                        .splineToLinearHeading(new Pose2d(-52, 28, -Math.PI / 2), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(SECONDX, SECONDY), -Math.PI / 2)

                        //GO TO BIG TRIANGLE

                        .setReversed(true)

                        .splineToSplineHeading(new Pose2d(-34, 23, -Math.PI), Math.toRadians(-90))

                        .stopAndAdd(
                                new SequentialAction(
                                        new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLING[1])),
                                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),
                                        robot.thirdShootSequence(),
                                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2]))
                                )
                        )

                        //THIRD INTAKE

                        .setReversed(false)

                        .splineToLinearHeading(new Pose2d(-76, 29, -Math.PI / 2), -Math.PI / 2)

                        .splineToConstantHeading(new Vector2d(-76, -7.5), -Math.PI / 2)


                        //GO TO BIG TRIANGLE

                        .setReversed(true)

                        .splineToSplineHeading(new Pose2d(-38, 23, -Math.PI), Math.toRadians(-90))

                        .stopAndAdd(
                                new SequentialAction(
                                        new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLING[1])),
                                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),
                                        robot.fourthShootSequence()
                                        //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2]))
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


                        robot.setFlywheelToCloseSideVelocity(),                   robot.updates(),

                        robot.antiTransfer(),
                        mainPath
                )
        );
        //SHOOT!
        telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
        telemetry.update();


    }
}
