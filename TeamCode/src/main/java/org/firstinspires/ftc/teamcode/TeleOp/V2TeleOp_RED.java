package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.ShooterSystems.PIPELINES;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.CommandUtils.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@TeleOp (name = "V2TeleOp_RED", group = "AAAA_MatchPurpose")
public class V2TeleOp_RED extends TeleOpBaseOpMode {

    public static PIPELINES PIPELINE = PIPELINES.RED_PIPELINE;

    private final RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    private final Intake intake = new Intake();

    private final LiteralTransfer literalTransfer = new LiteralTransfer();

    private final Shooter shooter = new Shooter();

    private ElapsedTime universalTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here
        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
        intake.provideComponents(super.intake, liftPTO, intakeBeambreak, transferBeambreak, controller1, controller2);
        literalTransfer.provideComponents(transfer, transferBeambreak, controller1);
        shooter.provideComponents(flywheel, turret, hoodAngler, customDrive, rev9AxisImu, controller1, controller2);

        //setup lynx module
        setUpLynxModule();

        //telemetry.speak("SIX SEVEN");

        if (isStopRequested()) return;
        waitForStart();
        CommandScheduler.start();

        shooter.start(Goal.GoalCoordinates.RED);

        //run robot reset
        RobotResetter robotReset = new PostAutonomousRobotReset(this);

        while (opModeIsActive() && !isStopRequested()) {

            // clear data at start of loop
            clearCacheOfLynxModule();

            controller1.getInformation();
            controller2.getInformation();

            intake.update();
            literalTransfer.update();

            shooter.update();

            robotCentricDrive.update();

            //background action processes
            CommandScheduler.update();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Run speed percentage", "%.2f", TickrateChecker.getRunSpeedPercentage());

            telemetry.addData("hood position", shooter.hoodAngler.getPosition());

            telemetry.addData("flywheel velocity estimate", "%.0f", shooter.flywheel.getCurrentVelocityEstimate());
            telemetry.addData("flywheel real velocity", "%.0f", shooter.flywheel.getRealVelocity());
            telemetry.addData("flywheel target velocity", shooter.flywheel.getTargetVelocity());

            telemetry.addData("p", shooter.flywheel.p);
            telemetry.addData("i", shooter.flywheel.i);
            telemetry.addData("d", shooter.flywheel.d);
            telemetry.addData("v", shooter.flywheel.v);
            telemetry.addData("power", shooter.flywheel.getMotorPowers()[0]);

            telemetry.addData("turret position error", shooter.turret.getRawPositionError());

            telemetry.addData("Robot pose", "x: %.2f, y: %.2f, heading: %.2f", shooter.robotPose.position.x, shooter.robotPose.position.y, shooter.robotPose.heading.toDouble());

            telemetry.addData("REV 9-axis IMU heading", shooter.rev9AxisImuHeadingDeg());

            telemetry.addData("Lift Engaged", intake.getLiftEngaged());
            telemetry.addData("Lift Position", intake.getLiftPosition());
            telemetry.update();

        }

        //end
        closeLynxModule();

    }
}
