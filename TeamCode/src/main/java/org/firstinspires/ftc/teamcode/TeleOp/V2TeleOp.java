package org.firstinspires.ftc.teamcode.TeleOp;




import com.qualcomm.robotcore.eventloop.opmode.TeleOp;




import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.util.RobotResetter;




@TeleOp (name = "V2TeleOp")
public class V2TeleOp extends TeleOpBaseOpMode {




    private final Intake intake = new Intake();
    BetterGamepad controller1 = new BetterGamepad(super.controller1);
    BetterGamepad controller2 = new BetterGamepad(super.controller2);


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




            controller1.getInformation();
            controller2.getInformation();




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





