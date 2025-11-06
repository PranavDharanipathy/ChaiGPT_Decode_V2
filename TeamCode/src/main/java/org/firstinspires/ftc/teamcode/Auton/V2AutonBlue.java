    package org.firstinspires.ftc.teamcode.Auton;

    import com.acmerobotics.roadrunner.Action;
    import com.acmerobotics.roadrunner.InstantAction;
    import com.acmerobotics.roadrunner.ParallelAction;
    import com.acmerobotics.roadrunner.Pose2d;
    import com.acmerobotics.roadrunner.ProfileAccelConstraint;
    import com.acmerobotics.roadrunner.SequentialAction;
    import com.acmerobotics.roadrunner.SleepAction;
    import com.acmerobotics.roadrunner.TranslationalVelConstraint;
    import com.acmerobotics.roadrunner.Vector2d;
    import com.acmerobotics.roadrunner.ftc.Actions;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
    import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
    import org.firstinspires.ftc.teamcode.TeleOp.Intake;
    import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

    import org.firstinspires.ftc.teamcode.Constants;


    @Autonomous (name = "V2Auton", group = "AAAA_MatchPurpose",  preselectTeleOp = "V2TeleOp_BLUE")
    public class V2AutonBlue extends AutonomousBaseOpMode {


        public class RobotElements {


            public InstantAction setFlywheelToFarSideVelocity() {
                Intake intake = new Intake();

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

            Action first_intake =
                    new SequentialAction(
                            //MOVE TO FIRST INTAKE POINT

                            new ParallelAction(
                                    robot.intake(),
                                    drive.actionBuilder(initialPose)

                                            //TANGENT = 180
                                            .splineTo(new Vector2d(23, 41), Math.PI / 2,
                                                    new TranslationalVelConstraint(60), new ProfileAccelConstraint(-50, 50))
                                            .build()


                            )

                    );

            Action goal =
                    new SequentialAction(
                            drive.actionBuilder(initialPose)
                                    .setReversed(true)

                                    //TODO: MAke the spline angle(reverse) more accurate
                                    .splineTo(new Vector2d(-3, 3), Math.toRadians(0))
                                    .build(),
                            new ParallelAction(
                                    new InstantAction(() -> hoodAngler.setPosition(ShooterInformation.ShooterConstants.HOOD_FAR_POSITION)),
                                    robot.transferArtifact(),
                                    robot.setFlywheelToFarSideVelocity()
                            )
                    );

            Action second_intake =
                    new SequentialAction(
                            //MOVE TO SECOND INTAKE POINT
                            drive.actionBuilder(initialPose)

                                    .splineTo(new Vector2d(45, 49), Math.PI / 2)

                                    .build(),

                            //Actual intake while moving forward
                            new ParallelAction(
                                    drive.actionBuilder(initialPose)
                                            .setReversed(false)
                                            .lineToX(26,  null)
                                            .build(),
                                            robot.intake()

                            )
                    );

            Action big_triangle =
                    new SequentialAction(
                            //MOVE TO BIG TRIANGLE/GOAL
                            drive.actionBuilder(initialPose)
                                    .splineToConstantHeading(new Vector2d(70, 0), Math.toRadians(0))
                                    .build()
                    );


            Action third_intake =
                    new SequentialAction(
                            drive.actionBuilder(initialPose)
                                    .splineTo(new Vector2d(70, 41), Math.PI / 2)
                                    .build()

                    );




                    new ParallelAction(
                            robot.intake()


                    );










            waitForStart();

            if (isStopRequested()) return;



            Actions.runBlocking(new SequentialAction(first_intake, goal, second_intake, goal));

            //SHOOT!





                telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
                telemetry.update();


                }





        }



