package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;

@Config
@Autonomous(name = "TurretPosition")
public class TurretPosition extends AutonomousBaseOpMode {


    public double TURRET_POSITION = 0;

    //0.9 = hood fully back

    //0.11 = hood fully forward
    public double[] HOOD_ANGLING = {0.11, 0.9};

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
                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() > 23500 && flywheel.getLastFrontendCalculatedVelocity() > 23500);
            }


        }


        public Action updates() {
            return new AllUpdate();
        }

        public InstantAction setFlywheelToCloseSideVelocity() {

            return new InstantAction(() -> flywheel.setVelocity(25600, true));
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

                    waitFlywheelVel(6),
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

                    waitFlywheelVel(2),
                    transferArtifact(),
                    new SleepAction(0.4),
                    antiTransfer(),

                    waitFlywheelVel(2),
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
            );
        }

    }

    public double turretStartPosition;

        @Override
        public void runOpMode() throws InterruptedException {

            init();

            final RobotElements robot = new RobotElements();

            turret.setPosition(turretStartPosition + TURRET_POSITION);



            new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLING[0]));

            robot.setFlywheelToCloseSideVelocity();

            robot.firstShootSequence();


            turretStartPosition = turret.getCurrentPosition();
            waitForStart();

            robot.updates();

            turret.update();


            if (isStopRequested()) return;



        }




    }

