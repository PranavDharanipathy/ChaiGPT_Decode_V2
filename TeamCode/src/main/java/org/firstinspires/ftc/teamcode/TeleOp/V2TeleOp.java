package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@TeleOp (name = "V2TeleOp")
public class V2TeleOp extends TeleOpBaseOpMode {

    private final Intake intake = new Intake();

    private final LiteralTransfer literalTransfer = new LiteralTransfer();

    @Override
    public void runOpMode() {

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here
        intake.provideComponents(super.intake, intakeBeambreak, transferBeambreak, controller1);
        literalTransfer.provideComponents(transfer, transferBeambreak, controller1);

        //setup lynx module
        setUpLynxModule();

        if (isStopRequested()) return;
        waitForStart();

        //run robot reset
        RobotResetter robotReset = new PostAutonomousRobotReset();

        while (opModeIsActive()) {

            TickrateChecker.startOfLoop();

            // clear data at start of loop
            clearCacheOfLynxModule();

            controller1.getInformation();
            controller2.getInformation();

            intake.update();
            literalTransfer.update();

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
