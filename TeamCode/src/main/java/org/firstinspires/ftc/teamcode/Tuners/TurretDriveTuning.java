package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.ShooterSystems.Goal;
import org.firstinspires.ftc.teamcode.ShooterSystems.PIPELINES;
import org.firstinspires.ftc.teamcode.TeleOp.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.LiteralTransfer;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.TeleOp.Shooter;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp (group = "tuning")
public class TurretDriveTuning extends TeleOpBaseOpMode {

    //turret tuning
    public static double KP = Constants.TURRET_PIDFS_COEFFICIENTS[0];
    public static double KI = Constants.TURRET_PIDFS_COEFFICIENTS[1];
    public static double KD = Constants.TURRET_PIDFS_COEFFICIENTS[2];
    public static double KS = Constants.TURRET_PIDFS_COEFFICIENTS[4];

    public static double KI_SMASH = Constants.TURRET_PIDFS_COEFFICIENTS[5];

    public static double KD_FILTER = Constants.TURRET_PIDFS_COEFFICIENTS[6];
    public static double KPOWER_FILTER = Constants.TURRET_PIDFS_COEFFICIENTS[7];

    public static double LANYARD_EQUILIBRIUM = Constants.TURRET_PIDFS_COEFFICIENTS[8];

    public static double MIN_I = Constants.TURRET_MIN_INTEGRAL_LIMIT, MAX_I = Constants.TURRET_MAX_INTEGRAL_LIMIT;




    private final RobotCentricDrive robotCentricDrive = new RobotCentricDrive();

    private final Intake intake = new Intake();

    private final LiteralTransfer literalTransfer = new LiteralTransfer();

    private final Shooter shooter = new Shooter();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here
        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
        intake.provideComponents(super.intake, intakeBeambreak, transferBeambreak, controller1);
        literalTransfer.provideComponents(transfer, transferBeambreak, controller1);
        shooter.provideComponents(flywheel, turret, hoodAngler, customDrive, rev9AxisImu, controller1, controller2);

        //setup lynx module
        setUpLynxModule();

        //telemetry.speak("SIX SEVEN");

        if (isStopRequested()) return;
        waitForStart();

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

            shooter.turret.setIConstraints(MIN_I, MAX_I);
            shooter.turret.updateCoefficients(KP, KI, KD, null, KS, KI_SMASH, KD_FILTER, KPOWER_FILTER, LANYARD_EQUILIBRIUM);
            shooter.update();

            robotCentricDrive.update();

            //background action processes

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Run speed percentage", "%.2f", TickrateChecker.getRunSpeedPercentage());

            telemetry.addData("hood position", shooter.hoodAngler.getPosition());

            telemetry.addData("flywheel current velocity", shooter.flywheel.getFrontendCalculatedVelocity());
            telemetry.addData("flywheel target velocity", shooter.flywheel.getTargetVelocity());

            telemetry.addData("p", shooter.turret.p);
            telemetry.addData("i", shooter.turret.i);
            telemetry.addData("d", shooter.turret.d);
            telemetry.addData("f", shooter.turret.f);
            telemetry.addData("s", shooter.turret.s);
            telemetry.addData("power", shooter.turret.getServoPowers()[0]);

            telemetry.addData("turret position error", shooter.turret.getRawPositionError());

            telemetry.addData("Robot pose", "x: %.2f, y: %.2f, heading: %.2f", shooter.robotPose.position.x, shooter.robotPose.position.y, shooter.robotPose.heading.toDouble());

            telemetry.addData("REV 9-axis IMU heading", shooter.rev9AxisImuHeadingDeg());
            telemetry.update();

        }

        //end
        closeLynxModule();

    }
}
