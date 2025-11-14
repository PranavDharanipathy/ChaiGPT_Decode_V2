package org.firstinspires.ftc.teamcode.Auton;




import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
@Autonomous (name = "V2AutonLONG(RED)", group = "AAAA_MatchPurpose",  preselectTeleOp = "V2TeleOp_RED")
public class V2AutonLONG_RED extends AutonomousBaseOpMode {


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


        public class FirstIntake implements Action {


            private ElapsedTime timer = new ElapsedTime();


            private boolean isIntakeStarted = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if (!isIntakeStarted) {
                    intake.setVelocity(Constants.BASE_INTAKE_VELOCITY);
                    isIntakeStarted = true;
                }


                if (timer.milliseconds() < 10_000) {
                    return true;
                }
                else return false;
            }
        }


        public Action firstIntake() {
            return new FirstIntake();
        }


        public class FirstShoot implements Action {


            private double[] SHOOT_TIMES = {
                    5000, 5300,
                    5700, 6000,
                    6400, 6700
            };


            private ElapsedTime timer = new ElapsedTime();


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                double t = timer.milliseconds();


                if (t > 6750) {
                    transfer.setVelocity(Constants.ANTI_TRANSFER_VELOCITY);
                    return false;
                }
                else {

                    if (
                            t >= SHOOT_TIMES[0] && t <= SHOOT_TIMES[1] ||
                                    t >= SHOOT_TIMES[2] && t <= SHOOT_TIMES[3] ||
                                    t >= SHOOT_TIMES[4] && t <= SHOOT_TIMES[5]
                    ) {
                        transfer.setVelocity(Constants.TRANSFER_VELOCITY);
                    }
                    else {
                        transfer.setVelocity(Constants.ANTI_TRANSFER_VELOCITY);
                    }


                    return true;
                }
            }
        }


        public Action firstShoot() {
            return new FirstShoot();
        }


    }


    public double turretStartPosition;


    @Override
    public void runOpMode() throws InterruptedException {


        fullInit();


        final RobotElements robot = new RobotElements();


        turretStartPosition = turret.getCurrentPosition();
        telemetry.addData("turret current position", turretStartPosition);
        telemetry.update();


        ElapsedTime timer = new ElapsedTime();
        int startPosition = 0;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();


        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Actions.runBlocking(robot.setFlywheelToFarSideVelocity());
        hoodAngler.setPosition(ShooterInformation.ShooterConstants.HOOD_FAR_POSITION);




        Action mainPath =


                new ParallelAction(


                        robot.antiTransfer(),


                        robot.firstIntake(),
                        robot.firstShoot(),


                        drive.actionBuilder(initialPose)
                                //PRELOAD POSITION TODO: Te,st and make angle more accurate
                                //   .splineToSplineHeading(new Pose2d(5, 4, Math.toRadians(140)), 0, new TranslationalVelConstraint(90), new ProfileAccelConstraint(-70, 70))
                                //START


                                //FIRST INTAKE


                                .splineTo(new Vector2d(-33, 57), Math.PI /2,
                                        new TranslationalVelConstraint(90), new ProfileAccelConstraint(-70, 70))


                                //SMALL TRIANGLE
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(43, -21, Math.toRadians(135)), Math.PI / 2,
                                        new TranslationalVelConstraint(95), new ProfileAccelConstraint(-50, 50))
                                .splineToSplineHeading(new Pose2d(23, -14, Math.PI), Math.PI / 2,
                                        new TranslationalVelConstraint(60), new ProfileAccelConstraint(-33, 33))






                                //SECOND INTAKE
                                //(52, -44)
                                .splineTo(new Vector2d(74, -54), Math.toRadians(-180),
                                        new TranslationalVelConstraint(90), new ProfileAccelConstraint(-70, 70))


                                //.splineToSplineHeading(new Pose2d(75, -54, Math.toRadians(-90)), Math.toRadians(-180),
                                //      new TranslationalVelConstraint(90), new ProfileAccelConstraint(-70, 70))
                                //.splineToSplineHeading(new Pose2d(69, -49, Math.toRadians(-90)), Math.toRadians(-90),
                                //new TranslationalVelConstraint(90), new ProfileAccelConstraint(-50, 50))


                                //GO TO SMALL TRIANGLE
                                //SMALL TRIANGLE
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(43, -21, Math.toRadians(135)), Math.PI / 2,
                                        new TranslationalVelConstraint(95), new ProfileAccelConstraint(-50, 50))
                                .splineToSplineHeading(new Pose2d(22, -13, Math.PI), Math.PI / 2,
                                        new TranslationalVelConstraint(60), new ProfileAccelConstraint(-33, 33))



                                .build()




                );










        waitForStart();


        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(mainPath));
        //SHOOT!
        telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
        telemetry.update();
    }


}

