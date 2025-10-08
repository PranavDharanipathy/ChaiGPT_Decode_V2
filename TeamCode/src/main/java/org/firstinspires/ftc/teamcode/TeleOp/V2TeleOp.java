package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;

public class V2TeleOp extends TeleOpBaseOpMode {

    private BetterGamepad gamepad1;
    private BetterGamepad gamepad2;

    @Override
    public void runOpMode() {

        initializeDevices();

        gamepad1 = new BetterGamepad(super.gamepad1);
        gamepad2 = new BetterGamepad(super.gamepad2);

        applyComponentTraits();

        //initialize subsystems here


        //setup lynx module
        setUpLynxModule();

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {

            TickrateChecker.startOfLoop();

            // clear data at start of loop
            clearCacheOfLynxModule();



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
