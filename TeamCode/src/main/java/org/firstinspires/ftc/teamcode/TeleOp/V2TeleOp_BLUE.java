package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.ShooterSystems.PIPELINES;
import org.firstinspires.ftc.teamcode.TeleOp.drive.NormalDrive;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp (name = "V2TeleOp BLUE", group = "AAAA_MatchPurpose")
public class V2TeleOp_BLUE extends TeleOpBaseOpMode {

    public static PIPELINES PIPELINE = PIPELINES.BLUE_PIPELINE;

    private final NormalDrive normalDrive = new NormalDrive();

    private final Intake intake = new Intake();

    private final LiteralTransfer literalTransfer = new LiteralTransfer();

    private final Shooter shooter = new Shooter();

    private ElapsedTime universalTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();

        applyComponentTraits();

        //initialize subsystems here
        normalDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);
        intake.provideComponents(super.intake, intakeBeambreak, transferBeambreak, controller1);
        literalTransfer.provideComponents(transfer, transferBeambreak, controller1);
        shooter.provideComponents(flywheel, turret, hoodAngler, unstartedLimelight, controller1, controller2);

        //setup lynx module
        setUpLynxModule();

        shooter.setPipeline(PIPELINE.getPipelineIndex());

        //telemetry.speak("SIX SEVEN");

        if (isStopRequested()) return;
        waitForStart();

        shooter.start();

        //run robot reset
        RobotResetter robotReset = new PostAutonomousRobotReset();

        while (opModeIsActive() && !isStopRequested()) {

            // clear data at start of loop
            clearCacheOfLynxModule();

            controller1.getInformation();
            controller2.getInformation();

            intake.update();
            literalTransfer.update();

            shooter.update();

            normalDrive.update();

            //background action processes

            telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
            telemetry.addData("(Predicted) Run speed percentage", "%.2f", TickrateChecker.getRunSpeedPercentage());

            telemetry.addData("hood position", shooter.hoodAngler.getPosition());

            telemetry.addData("flywheel current velocity", shooter.flywheel.getFrontendCalculatedVelocity());
            telemetry.addData("flywheel target velocity", shooter.flywheel.getTargetVelocity());

            telemetry.addData("p", shooter.flywheel.p);
            telemetry.addData("i", shooter.flywheel.i);
            telemetry.addData("d", shooter.flywheel.d);
            telemetry.addData("v", shooter.flywheel.v);
            telemetry.addData("power", shooter.flywheel.$getMotorPowers()[0]);


            telemetry.addData("limelight result", shooter.llResult);
            telemetry.addData("is limelight result valid?", shooter.llResult.isValid());

            telemetry.addData("limelight tx", shooter.llResult.getTx());
            telemetry.addData("adjusted tx", shooter.getAdjustedTx());
            telemetry.update();

        }

        //end
        closeLynxModule();

    }
}
