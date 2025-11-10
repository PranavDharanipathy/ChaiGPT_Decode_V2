package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "V2AutonCLOSE_Blue", group = "AAAA_MatchPurpose", preselectTeleOp = "V2Teleop_BLUE")
public class V2AutonCLOSE_Blue extends AutonomousBaseOpMode {

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

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action mainPath =
                new ParallelAction(
                        robot.intake(),
                        drive.actionBuilder(initialPose)
                               /// TODO: 11/9/2025 MAKE PRELOAD SPLINE UPDATE: CHECK PRELOAD

                                .splineToSplineHeading(new Pose2d(-35, -64, -45), Math.toRadians(-90))

                               //FIRST INTAKE

                                //.splineTo(new Vector2d(-21, -27), -Math.PI / 2)
                                .splineTo(new Vector2d(-42, 0), -Math.PI / 2)

                                //GO TO shooting spot(big triangle)
                                .splineToSplineHeading(new Pose2d(-35, -64, -45), Math.toRadians(-90))

                                .waitSeconds(4)

                                //SECOND INTAKE
                                .splineTo(new Vector2d(-65, -0), 90)

                                //GO TO shooting spot(big triangle)

                                .splineToSplineHeading(new Pose2d(70, 0, -45), Math.toRadians(-90))
                                .waitSeconds(4)

                                //THIRD INTAKE






                                .build()

                );


                waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(new ParallelAction(robot.antiTransfer(), mainPath));
        //SHOOT!
        telemetry.addData("flywheel speed", flywheel.getFrontendCalculatedVelocity());
        telemetry.update();
    }

}

