package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@TeleOp (name = "V2TeleOp")
public class V2TeleOp extends TeleOpBaseOpMode {

    private BetterGamepad gamepad1;
    private BetterGamepad gamepad2;

    private Intake intake = new Intake();

    @Override
    public void runOpMode() {

        initializeDevices();

        gamepad1 = new BetterGamepad(super.gamepad1);
        gamepad2 = new BetterGamepad(super.gamepad2);

        applyComponentTraits();

        //initialize subsystems here
        intake.provideComponents(super.intake, gamepad1);

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

            gamepad1.getInformation();
            gamepad2.getInformation();

            intake.update();

            TickrateChecker.endOfLoop();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.update();

            if(isStopRequested()) {
                //end
                closeLynxModule();
            }
        }
    }
}
