package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous (name = "ThreeBallAuto RED FAR", group = "AAAA_MatchPurpose", preselectTeleOp = "V2TeleOp RED")
public class ThreeBallAuto_RED_FAR extends AutonomousBaseOpMode {

    public class RobotElements {

        public InstantAction setFlywheelToFarSideVelocity() {
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
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0,0,0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        fullInit();

        ElapsedTime timer = new ElapsedTime();

        final RobotElements robot = new RobotElements();

        turretStartPosition = turret.getCurrentPosition();

        telemetry.addData("turret current position", turretStartPosition);
        telemetry.update();

        Action shootPreloadedArtifacts =

                new SequentialAction(
                        new ParallelAction(
                                robot.intake(),
                                robot.transferArtifact()
                        ),
                        new SleepAction(1),

                        new ParallelAction(
                                robot.intake(),
                                robot.transferArtifact()
                        ),
                        new SleepAction(1),

                        new ParallelAction(
                                robot.intake(),
                                robot.transferArtifact()
                        )
                );




        Actions.runBlocking(robot.setFlywheelToFarSideVelocity());

        waitForStart();

        //START!

        hoodAngler.setPosition(ShooterInformation.ShooterConstants.HOOD_FAR_POSITION);

        timer.reset();

        Actions.runBlocking(

                new ParallelAction(

                        telemetryPacket -> {
                            if (timer.milliseconds() > Constants.FLYWHEEL_PIDFVAS_LOOP_TIME) {

                                flywheel.update();
                                timer.reset();
                            }

                            if (opModeIsActive()) {
                                return true;
                            }
                            else {
                                flywheel.setVelocity(0, true);
                                return false;
                            }
                        },

                        robot.intake(),
                        robot.antiTransfer(),

                        new SequentialAction(
                                drive.actionBuilder(startPose)
                                        .splineToLinearHeading(new Pose2d(-16, 0, Math.toRadians(-25)), Math.toRadians(0))
                                        .build(),

                                telemetryPacket -> {

                                    telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                                    telemetry.update();
                                    return !(flywheel.getFrontendCalculatedVelocity() > 29000 && flywheel.getLastFrontendCalculatedVelocity() > 29000);
                                },

                                robot.transferArtifact(),

                                new SleepAction(0.45),

                                robot.antiTransfer(),

                                new SleepAction(1), //forcefully increase wait time

                                telemetryPacket -> {

                                    telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                                    telemetry.update();
                                    return !(flywheel.getFrontendCalculatedVelocity() > 29000 && flywheel.getLastFrontendCalculatedVelocity() > 29000);
                                },

                                robot.transferArtifact(),

                                new SleepAction(0.3),

                                robot.antiTransfer(),

                                telemetryPacket -> {

                                    telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                                    telemetry.update();
                                    return !(flywheel.getFrontendCalculatedVelocity() > 29000 && flywheel.getLastFrontendCalculatedVelocity() > 29000);
                                },

                                robot.transferArtifact()

                        )
                )
        );



    }

}
