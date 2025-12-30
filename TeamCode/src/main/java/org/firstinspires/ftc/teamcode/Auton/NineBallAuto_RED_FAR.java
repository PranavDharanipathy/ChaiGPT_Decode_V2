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
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;


import org.firstinspires.ftc.teamcode.TeleOp.Shooter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


import org.firstinspires.ftc.teamcode.Constants;

@Config
@Autonomous (name = "NineBallAuto RED FAR", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp_RED")
public class NineBallAuto_RED_FAR extends AutonomousBaseOpMode {


    /// Turret Positin Tuning Instructions:
    /// 1: Run this code(without changing anything)
    /// 2: While the code is running, make a mental note of what needs to be changed(Do one thing at a time)
    /// 3: After you run the code, DONT change the values in this code and redownload; that takes too much time.
    /// 3a: Go to the FTC dashboard(Make sure you are connected to the bot through WIFI), and look for the name of this program("NineBallAuto BLUE FAR")
    /// 3b: Click on the TURRET_POSITIONS dropdown and edit the values there
    /// 3c: Do the same for SET_FLYWHEEL_VELOCITY and SHOOTING_VELOCITY(definitions below in @param)

    /// DEFINITIONS OF VALUES:

    /// @param turret_positions --> turret position of bot, negative value turns turret to right, positive value turns turret to left
    /// @param SET_FLYWHEEL_VELOCITY --> The robot will set this base flywheel velocity, set this higher than your actual shooting velocity
    /// @param SHOOTING_VELOCITY --> The robot will shoot once it reaches this velocity(different from SET_FLYWHEEL_VELOCITY)

    /// If 2 balls shoot and the 3rd one just stays in Auton, msg me(Nikhil) on discord


    public static double[] TURRET_POSITIONS = {1200, -1950, 1370};

    public static double SET_FLYWHEEL_VELOCITY = 30500;

    public static double SHOOTING_VELOCITY = 37000;


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

            //old: ShooterInformation.ShooterConstants.FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY

            return new InstantAction(() -> flywheel.setVelocity(SET_FLYWHEEL_VELOCITY, true));
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
            return new InstantAction(() -> intake.setVelocity(Constants.REVERSE_INTAKE_POWER));
        }


        public InstantAction intake() {
            return new InstantAction(() -> intake.setVelocity(Constants.INTAKE_POWER));
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
                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() > SHOOTING_VELOCITY && flywheel.getLastFrontendCalculatedVelocity() > SHOOTING_VELOCITY);
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

                    waitTilFlywheelAtVelocity(2.5),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(2.5),
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

                    waitTilFlywheelAtVelocity(3.5),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitTilFlywheelAtVelocity(3.5),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    //setup for third
                    new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2]))
            );
        }

        public Action thirdShootSequence() {

            return new SequentialAction(
//                    new InstantAction(() -> hoodAngler.setPosition(0.11)),
                    waitTilFlywheelAtVelocity(4),
                    transferArtifact(),
                    new SleepAction(0.3),
                    antiTransfer(),

//                    new InstantAction(() -> hoodAngler.setPosition(0.112)),
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
                                .splineToLinearHeading(new Pose2d(-7, 7, Math.toRadians(-36)), 0)

                                .stopAndAdd(robot.firstShootSequence())

                                //first intake

                                .splineToLinearHeading(new Pose2d(-33, 7.5, Math.PI / 2), Math.toRadians(-36))
                                .waitSeconds(0.1)
                                .splineToConstantHeading(new Vector2d(-33, 41), Math.PI / 2)

                                //GO TO SMALL TRIANGLE
                                .setReversed(true)


                                .splineToSplineHeading(new Pose2d(-13, 7, 0), Math.PI / 2)


                                .stopAndAdd(robot.secondShootSequence())


                                .setReversed(false)


                                ///.splineToSplineHeading(new Pose2d(-44, 0, -Math.PI / 2), 0,
                                //new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))


                                //SECOND INTAKE


                                .splineToLinearHeading(new Pose2d(-56, 8, Math.PI / 2), 0)
                                .waitSeconds(0.1)
                                .splineToConstantHeading(new Vector2d(-56, 35), -Math.PI / 2)


                                //GO TO SMALL TRIANGLE


                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-38, 30), Math.PI / 2)
                                .splineToSplineHeading(new Pose2d(-7, 7, Math.toRadians(-36)), Math.PI / 2)

                                .stopAndAdd(
                                        new SequentialAction(

                                                robot.thirdShootSequence(),
                                                new InstantAction(() -> turret.setPosition(turretStartPosition))
                                        )
                                )

                                //movement rp
                                .splineToLinearHeading(new Pose2d(-20, 12, Math.toRadians(0)), Math.toRadians(-36))
                                .build());






        turretStartPosition = turret.getCurrentPosition();
        telemetry.addData("turret current position", turretStartPosition);
        telemetry.update();


        waitForStart();


        if (isStopRequested()) return;






        Actions.runBlocking(
                new ParallelAction(

                        new InstantAction(() -> hoodAngler.setPosition(0.112)),

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