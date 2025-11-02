    package org.firstinspires.ftc.teamcode.Auton;

    import com.acmerobotics.roadrunner.Action;
    import com.acmerobotics.roadrunner.InstantAction;
    import com.acmerobotics.roadrunner.ParallelAction;
    import com.acmerobotics.roadrunner.Pose2d;
    import com.acmerobotics.roadrunner.SequentialAction;
    import com.acmerobotics.roadrunner.Trajectory;
    import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
    import com.acmerobotics.roadrunner.Vector2d;
    import com.acmerobotics.roadrunner.ftc.Actions;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
    import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
    import org.firstinspires.ftc.teamcode.TeleOp.Intake;
    import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

    import org.firstinspires.ftc.teamcode.Constants;

    import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;



    @Autonomous (name = "V2Auton", preselectTeleOp = "V2TeleOp_BLUE")
    public abstract class V2Auton extends AutonomousBaseOpMode {

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

            Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            Action first_intake =
                    new SequentialAction(
                            //MOVE TO FIRST INTAKE POINT
                            drive.actionBuilder(initialPose)
                                    .splineToLinearHeading(new Pose2d(42, 54, Math.toRadians(-90)), Math.toRadians(0))

                                            .build(),

                            //Actual intake
                            new ParallelAction(
                                    drive.actionBuilder(initialPose)
                                            .lineToX(26,  null)
                                            .build(),
                                            new SequentialAction(
                                                    robot.intake()

                                            )

                            )

                    );

            Action go_to_goal =
                    new SequentialAction(
                            drive.actionBuilder(initialPose)
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(21, 43, Math.toRadians(-90)), Math.toRadians(0))
                                    .strafeTo(new Vector2d(-21,50))
                                        .build()
                    );

            Action second_intake =
                    new SequentialAction(
                            //MOVE TO FIRST INTAKE POINT
                            drive.actionBuilder(initialPose)
                                    .splineToLinearHeading(new Pose2d(42, 54, Math.toRadians(-90)), Math.toRadians(0))

                                    .build(),

                            //Actual intake
                            new ParallelAction(
                                    drive.actionBuilder(initialPose)
                                            .lineToX(26,  null)
                                            .build(),
                                    new SequentialAction(
                                            robot.intake()

                                    )

                            )

                    );





            waitForStart();

            if (isStopRequested()) return;

            Actions.runBlocking(first_intake);
            Actions.runBlocking(go_to_goal);

            //SHOOT!

            hoodAngler.setPosition(ShooterInformation.ShooterConstants.HOOD_FAR_POSITION);
            robot.transferArtifact();

            robot.setFlywheelToFarSideVelocity();





        }


    }
