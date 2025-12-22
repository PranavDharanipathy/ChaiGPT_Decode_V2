package org.firstinspires.ftc.teamcode.Tuners.DriveTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp (group = "tuning")
public class ShooterDriveTuning extends TeleOpBaseOpMode {

    //shooter tuning
    public static double TRANSFER_VELOCITY = 1200;
    public static double INTAKE_POWER = 1;
    public static double FLYWHEEL_VELOCITY = 405_000;
    public static double HOOD_POSITION = 0.2;



    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();

        applyComponentTraits();

        //setup lynx module
        setUpLynxModule();

        //telemetry.speak("SIX SEVEN");

        if (isStopRequested()) return;
        waitForStart();

        //run robot reset
        RobotResetter robotReset = new PostAutonomousRobotReset(this);

        while (opModeIsActive() && !isStopRequested()) {

            // clear data at start of loop
            clearCacheOfLynxModule();

            hoodAngler.setPosition(HOOD_POSITION);
            intake.setPower(INTAKE_POWER);
            transfer.setVelocity(TRANSFER_VELOCITY);
            flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
            flywheel.updateKvBasedOnVoltage();
            flywheel.update();

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Run speed percentage", "%.2f", TickrateChecker.getRunSpeedPercentage());

            telemetry.addData("hood position", hoodAngler.getPosition());

            telemetry.addData("p", flywheel.p);
            telemetry.addData("i", flywheel.i);
            telemetry.addData("d", flywheel.d);
            telemetry.addData("v", flywheel.v);

            telemetry.addData("flywheel current velocity", flywheel.getFrontendCalculatedVelocity());
            telemetry.addData("flywheel target velocity", flywheel.getTargetVelocity());

            telemetry.addData("turret current position", turret.getCurrentPosition());
            telemetry.addData("turret target position", turret.getTargetPosition());

            telemetry.update();

        }

        //end
        closeLynxModule();

    }
}
