package org.firstinspires.ftc.teamcode.Auton;


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


import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;


import org.firstinspires.ftc.teamcode.TeleOp.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


import org.firstinspires.ftc.teamcode.Constants;


@Autonomous (name = "V2AutonLONG(BLUE)", group = "AAAA_MatchPurpose", preselectTeleOp = "V2Teleop_BLUE")
public class V2AutonLONG_BLUE extends AutonomousBaseOpMode {




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


        public class WaitTilFlywheelAtVelocity implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                telemetry.update();
                return !(flywheel.getFrontendCalculatedVelocity() > 29000 && flywheel.getLastFrontendCalculatedVelocity() > 33000);
            }
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
                        drive.actionBuilder(initialPose)




                                //TANGENT = 90
                                //FIRST INTAKE
                                .splineTo(new Vector2d(-21, -52), -Math.PI / 2)




                                //GO TO SMALL TRIANGLE
                                .setReversed(true)


                                .splineToSplineHeading(new Pose2d(-15, -6, 0), -Math.PI / 2)


                                .stopAndAdd(
                                        new SequentialAction(
                                                robot.transferArtifact()



                                                )
                                )


                                .setReversed(false)


                                ///.splineToSplineHeading(new Pose2d(-44, 0, -Math.PI / 2), 0,
                                //new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))


                                //SECOND INTAKE


                                .splineToSplineHeading(new Pose2d(-43, -19, -Math.PI / 2), 0)
                                .splineToConstantHeading(new Vector2d(-44, -59), -Math.PI / 2)


                                //GO TO SMALL TRIANGLE


                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-38, -30), -Math.PI / 2)
                                .splineToSplineHeading(new Pose2d(-18, -5, Math.toRadians(36)), -Math.PI / 2)
//1.5 secs
                                .build());






        turretStartPosition = turret.getCurrentPosition();
        telemetry.addData("turret current position", turretStartPosition);
        telemetry.update();


        waitForStart();


        if (isStopRequested()) return;






        Actions.runBlocking(
                new ParallelAction(


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

