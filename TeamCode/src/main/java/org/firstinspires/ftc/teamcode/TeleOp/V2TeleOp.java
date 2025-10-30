package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.TeleOp.drive.NormalDrive;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp (name = "V2TeleOp")
public class V2TeleOp extends TeleOpBaseOpMode {

    private final NormalDrive normalDrive = new NormalDrive();

    private final Intake intake = new Intake();

    private final LiteralTransfer literalTransfer = new LiteralTransfer();

    private final Shooter shooter = new Shooter();

    public static int PIPELINE = 2;

    @Override
    public void runOpMode() {

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here
        normalDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
        intake.provideComponents(super.intake, intakeBeambreak, transferBeambreak, controller1);
        literalTransfer.provideComponents(transfer, transferBeambreak, controller1);
        shooter.provideComponents(flywheel, turret, unstartedLimelight, controller1, controller2);

        //setup lynx module
        setUpLynxModule();

        if (isStopRequested()) return;
        waitForStart();

        shooter.start();

        //run robot reset
        RobotResetter robotReset = new PostAutonomousRobotReset();

        while (opModeIsActive()) {

            TickrateChecker.startOfLoop();

            // clear data at start of loop
            clearCacheOfLynxModule();

            controller1.getInformation();
            controller2.getInformation();

            shooter.setPipeline(PIPELINE);

            intake.update();
            literalTransfer.update();

            shooter.update();

            normalDrive.update();

            //background action processes

            TickrateChecker.endOfLoop();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Run speed percentage", "%.2f", TickrateChecker.getRunSpeedPercentage());
            telemetry.update();

            if(isStopRequested()) {
                //end
                closeLynxModule();
            }
        }
    }
}
