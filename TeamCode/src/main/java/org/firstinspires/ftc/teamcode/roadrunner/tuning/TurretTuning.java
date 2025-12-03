package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;


@Config
@TeleOp(name = "Turret Tuning", group="tuning")
public class TurretTuning extends AutonomousBaseOpMode {
    public static double TURRET_POSITION = 0;

    public static double[] HOOD_ANGLING = {0.11, 0.9};

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

                return true;
            }
        }

        public class FlywheelWaitVel implements Action {
            private ElapsedTime timer = new ElapsedTime();

            private double minimumTime;

            public FlywheelWaitVel(double minimumTime) {
                this.minimumTime = minimumTime;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                telemetry.update();
                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() > 25500 && flywheel.getLastFrontendCalculatedVelocity() > 25500);
            }


        }


        public Action updates() {
            return new RobotElements.AllUpdate();
        }

        public void setFlywheelToCloseSideVelocity() {

            new InstantAction(() -> flywheel.setVelocity(28600, true));
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


        public Action waitFlywheelVel(double minimumTime) {
            return new FlywheelWaitVel(minimumTime);
        }

        //public double turretStartPosition;
        public Action firstShootSequence() {

            return new SequentialAction(
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[2])),
                    new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLING[0])),

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

                    //setup for second
                    //new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITIONS[0]))
            );
        }

    }




        public double turretStartPosition;



    @Override
    public void runOpMode() throws InterruptedException {

        final RobotElements robot = new RobotElements();
        //Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-225));
        //MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITION));

        robot.setFlywheelToCloseSideVelocity();

        new InstantAction(() -> hoodAngler.setPosition(HOOD_ANGLING[0]));

                robot.waitFlywheelVel(3);
                robot.transferArtifact();
                new SleepAction(0.4);
                robot.antiTransfer();






        turretStartPosition = turret.getCurrentPosition();

        //SHOOT!
        telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
        telemetry.update();

        waitForStart();


        }
    }


