package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.CommandUtils.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp (name = "V2TeleOp_TEST")
public class V2TeleOp_TEST extends TeleOpBaseOpMode {

    public static CurrentAlliance alliance = new CurrentAlliance(CurrentAlliance.ALLIANCE.BLUE_ALLIANCE);

    private final PedroDrive pedroDrive = new PedroDrive();

    private final Intake intake = new Intake();

    private final LiteralTransfer literalTransfer = new LiteralTransfer();

    private final Shooter shooter = new Shooter();

    private final TelemetrySubsystem telemetry = new TelemetrySubsystem();

    //private ElapsedTime universalTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here
        telemetry.provideComponents(super.telemetry, true, controller2);
        pedroDrive.provideInitComponents(follower, controller1, controller2, alliance);
        intake.provideComponents(super.intake, liftPTO, intakeBeambreak, transferBeambreak, controller1, controller2);
        literalTransfer.provideComponents(transfer, transferBeambreak, controller1);
        shooter.provideComponents(flywheel, turret, hoodAngler, follower, rev9AxisImu, controller1, controller2);

        //setup lynx module
        setUpLynxModule();

        if (isStopRequested()) return;
        waitForStart();
        CommandScheduler.start();

        shooter.start(Goal.GoalCoordinates.BLUE);

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

            pedroDrive.provideLoopComponents(intake.getStage());
            pedroDrive.update();

            //background action processes
            CommandScheduler.update();

            telemetry.runInstance(shooter, intake, pedroDrive);
        }

        if(isStopRequested()) {
            //end
            closeLynxModule();
        }

    }
}
