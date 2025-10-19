package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PIDFControlDrive;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@TeleOp (name = "V2TeleOp")
public class V2TeleOp extends TeleOpBaseOpMode {

    private final Intake intake = new Intake();

    @Override
    public void runOpMode() {

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here

        BasicVeloMotor intakeMotor = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.intakeMotorDeviceName);
        BasicVeloMotor transferMotor = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.transferMotorDeviceName);


        AdafruitBeambreakSensor intakeSensor = new AdafruitBeambreakSensor(hardwareMap,
                Constants.MapSetterConstants.intakeBeambreakSensorNames[0],
                Constants.MapSetterConstants.intakeBeambreakSensorNames[1]);

        AdafruitBeambreakSensor transferSensor = new AdafruitBeambreakSensor(hardwareMap,
                Constants.MapSetterConstants.transferBeambreakSensorNames[0],
                Constants.MapSetterConstants.transferBeambreakSensorNames[1]);


        intake.provideComponents(super.intake, transfer, intakeBeambreak, transferBeambreak, controller1);


        //setup lynx module
        setUpLynxModule();

        if (isStopRequested()) return;
        waitForStart();

        //run robot reset


        while (opModeIsActive()) {

            TickrateChecker.startOfLoop();

            // clear data at start of loop
            clearCacheOfLynxModule();

            intake.update();


            controller1.getInformation();
            controller2.getInformation();

            TickrateChecker.endOfLoop();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Current CPU Usage", TickrateChecker.getTimeBasedCpuUsagePrediction());
            telemetry.update();

            if(isStopRequested()) {
                intakeMotor.setVelocity(0);
                transferMotor.setVelocity(0);
                closeLynxModule();
                //end
            }
        }
    }
}
