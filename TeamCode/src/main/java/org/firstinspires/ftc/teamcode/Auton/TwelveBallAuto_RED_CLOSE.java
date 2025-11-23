package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Vector;


@Config
@Autonomous (name = "12 Ball RED CLOSE(AUTO)", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp_RED")
public class TwelveBallAuto_RED_CLOSE extends AutonomousBaseOpMode {
    public static double[] TURRET_POSITIONS = {6400, 6700 , 5500};

    public static double[] HOOD_ANGLING = {0.31, 0.04, 0.06};
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
                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() > 21500 && flywheel.getLastFrontendCalculatedVelocity() > 21500);
            }


        }


        public Action updates() {
            return new RobotElements.AllUpdate();
        }

        public InstantAction setFlywheelToCloseSideVelocity() {

            return new InstantAction(() -> flywheel.setVelocity(26800, true));
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
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.4),
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
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(3),
                    transferArtifact(),
                    new SleepAction(0.4),
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
                    new SleepAction(0.3),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.3),
                    antiTransfer(),

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.3),
                    antiTransfer()
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
            );
        }

        public Action fourthShootSequence() {
            return new SequentialAction(
                    new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0])),
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
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action mainPath =
                new ParallelAction(
                        robot.intake(),
                        //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),

                        //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[1])),
                        drive.actionBuilder(initialPose)
                                //PRELOAD


                                .build()
                );


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
