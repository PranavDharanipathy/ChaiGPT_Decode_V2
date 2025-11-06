package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.AutonomousBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.TeleOp.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous (name = "V2Auton(RED)", group = "AAAA_MatchPurpose",  preselectTeleOp = "V2TeleOp_RED")
public class V2AutonRed extends AutonomousBaseOpMode {

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


        Action first_intake =
                new ParallelAction(
                        robot.intake(),
                        drive.actionBuilder(initialPose)
                                .splineTo(new Vector2d(28.5, 44), -90)
                                .build()
                );

        Action goal =
                new SequentialAction(
                        drive.actionBuilder(initialPose)
                                .setReversed(true)

                                //TODO: MAke the spline angle(reverse) more accurate
                                .splineTo(new Vector2d(-3, 1), Math.toRadians(0))
                                .build(),
                        new ParallelAction(
                                new InstantAction(() -> hoodAngler.setPosition(ShooterInformation.ShooterConstants.HOOD_FAR_POSITION)),
                                robot.transferArtifact(),
                                robot.setFlywheelToFarSideVelocity()
                        )
                );

        Action second_intake =
                new ParallelAction(
                        drive.actionBuilder(initialPose)
                                .splineTo(new Vector2d(52, 45), -90)
                                .build()
                );
    }

}
