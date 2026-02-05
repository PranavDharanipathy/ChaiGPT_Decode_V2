package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.CommandUtils.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp (name = "V2TeleOp_TEST")
public class V2TeleOp_TEST extends TeleOpBaseOpMode {

    //sorry for how inefficient this part is
    public static double RED_CLOSE_GOAL_X = Goal.GoalCoordinates.RED.getCloseCoordinate().getX();
    public static double RED_CLOSE_GOAL_Y = Goal.GoalCoordinates.RED.getCloseCoordinate().getY();
    public static double RED_FAR_GOAL_X = Goal.GoalCoordinates.RED.getFarCoordinate().getX();
    public static double RED_FAR_GOAL_Y = Goal.GoalCoordinates.RED.getFarCoordinate().getY();

    public static double BLUE_CLOSE_GOAL_X = Goal.GoalCoordinates.BLUE.getCloseCoordinate().getX();
    public static double BLUE_CLOSE_GOAL_Y = Goal.GoalCoordinates.BLUE.getCloseCoordinate().getY();
    public static double BLUE_FAR_GOAL_X =Goal.GoalCoordinates.BLUE.getFarCoordinate().getX();
    public static double BLUE_FAR_GOAL_Y =Goal.GoalCoordinates.BLUE.getFarCoordinate().getY();

    public static CurrentAlliance.ALLIANCE alliance = CurrentAlliance.ALLIANCE.BLUE_ALLIANCE;

    private CurrentAlliance currentAlliance = new CurrentAlliance(alliance);

    private final PedroDrive pedroDrive = new PedroDrive();

    private final Intake intake = new Intake();

    private final LiteralTransfer literalTransfer = new LiteralTransfer();

    private final Shooter shooter = new Shooter();

    private final TelemetrySubsystem telemetry = new TelemetrySubsystem();

    //private ElapsedTime universalTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        //useEOALocalizationData();

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here
        telemetry.provideComponents(super.telemetry, true, controller2);
        pedroDrive.provideInitComponents(follower, controller1, controller2, currentAlliance);
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

            pedroDrive.provideInitComponents(follower, controller1, controller2, currentAlliance);
            shooter.switchAlliance(alliance);

            if (alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE) {

                shooter.accessGoalCoordinates().setGoalCoordinates(
                        new Goal.GoalCoordinate(BLUE_CLOSE_GOAL_X, BLUE_CLOSE_GOAL_Y), //close
                        new Goal.GoalCoordinate(BLUE_FAR_GOAL_X, BLUE_FAR_GOAL_Y) //far
                );
            }
            else {

                shooter.accessGoalCoordinates().setGoalCoordinates(
                        new Goal.GoalCoordinate(RED_CLOSE_GOAL_X, RED_CLOSE_GOAL_Y), //close
                        new Goal.GoalCoordinate(RED_FAR_GOAL_X, RED_FAR_GOAL_Y) //far
                );
            }

            controller1.getInformation();
            controller2.getInformation();

            intake.update();
            literalTransfer.update();

            shooter.update();

            pedroDrive.provideLoopComponents(intake.getStage());
            pedroDrive.update();

            //background action processes
            CommandScheduler.update();

            super.telemetry.addData("alliance", currentAlliance.toString());
            telemetry.runInstance(shooter, intake, pedroDrive);
        }

        if(isStopRequested()) {
            //end
            closeLynxModule();
        }

    }

}
