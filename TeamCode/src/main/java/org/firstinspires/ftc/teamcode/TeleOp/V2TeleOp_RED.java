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

    private final RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

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

            telemetry.runInstance(shooter, intake);
        }

        //end
        closeLynxModule();

    }
}
