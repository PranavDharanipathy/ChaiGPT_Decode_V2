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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;


import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


import org.firstinspires.ftc.teamcode.Constants;

//@Config
@Autonomous (name = "ThreeBallAuto BLUE FAR", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp_BLUE")
public class ThreeBallAuto_BLUE_FAR extends AutonomousBaseOpMode {


    public static double TURRET_POSITION = -1300;


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
                telemetry.addData("flywheel current velocity", flywheel.getFrontendCalculatedVelocity());
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

                return !(timer.seconds() >= minimumTime && flywheel.getFrontendCalculatedVelocity() >= minimumVelocity);
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
                        new InstantAction(() -> turret.setPosition(turretStartPosition + TURRET_POSITION)),

                        drive.actionBuilder(initialPose)


                                //preload
                                .splineToLinearHeading(new Pose2d(-17, -6.5, Math.toRadians(36)), 0)

                                .stopAndAdd(robot.firstShootSequence())


                                //movement rp
                                .splineToLinearHeading(new Pose2d(-22, -10, Math.toRadians(0)), Math.toRadians(36))
                                .build());






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