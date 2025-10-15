package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PIDFControlDrive;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@TeleOp (name = "V2TeleOp")
public class V2TeleOp extends TeleOpBaseOpMode {

    private PIDFControlDrive drive = new PIDFControlDrive();

    @Override
    public void runOpMode() {

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here


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


            drive.update();

            controller1.getInformation();
            controller2.getInformation();

            TickrateChecker.endOfLoop();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Current CPU Usage", TickrateChecker.getTimeBasedCpuUsagePrediction());
            telemetry.update();

            if(isStopRequested()) {
                //end
                closeLynxModule();
            }
        }
    }
}
