package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.TeleOp.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.util.SubsystemInternal;
import org.firstinspires.ftc.teamcode.util.TelemetryUtils.TelemetryMode;
import org.firstinspires.ftc.teamcode.util.TelemetryUtils.TelemetryO;

public class TelemetrySubsystem implements SubsystemInternal {

    private TelemetryO telem;

    private BetterGamepad controller2;

    public void provideComponents(Telemetry telemetry, boolean addFTCDashboard, BetterGamepad controller2) {

        telem = new TelemetryO(telemetry, addFTCDashboard);

        telem.setTelemetryModes(TelemetryMode.INFO);

        this.controller2 = controller2;
    }

    public void runInstance(Shooter shooter, Intake intake, PedroDrive pedroDrive) {

        if (controller2.right_bumperHasJustBeenPressed) {

            telem.clearAll();

            if (telem.getTelemetryModes().contains(TelemetryMode.INFO)) {
                telem.setTelemetryModes(TelemetryMode.RAW_DATA);
            }
            else {
                telem.setTelemetryModes(TelemetryMode.INFO);
            }
        }

        telem.addData("Tick rate", TickrateChecker.getTimePerTick());
        telem.addData("(Predicted) Run speed percentage", "%.2f", TickrateChecker.getRunSpeedPercentage());

        telem.addData("EOA Pose", shooter.EOAPose.toString());

        telem.addData("zone", shooter.getZone().toString());
        telem.addData(TelemetryMode.RAW_DATA, "is alliance close?", shooter.accessGoalCoordinates().isAllianceClose(shooter.futureRobotPose.getY(), shooter.accessGoalCoordinates()));

        telem.addData("hood mode", shooter.usingAutomaticHood() ? "AUTOMATIC" : "STATIC");

        telem.addData(TelemetryMode.RAW_DATA, "hood position", shooter.hoodAngler.getPosition());

        telem.addData(TelemetryMode.RAW_DATA, "flywheel velocity estimate", "%.0f", shooter.flywheel.getCurrentVelocityEstimate());
        telem.addData(TelemetryMode.INFO, "flywheel real velocity", "%.0f", shooter.flywheel.getRealVelocity());
        telem.addData(TelemetryMode.INFO, "flywheel target velocity", shooter.flywheel.getTargetVelocity());

        telem.addData(TelemetryMode.INFO, "turret target angle", shooter.tt);
        telem.addData(TelemetryMode.INFO, "turret position error", shooter.turret.getRawPositionError());

        telem.addData(TelemetryMode.RAW_DATA, "current robot pose", "x: %.2f, y: %.2f, heading: %.2f", shooter.currentRobotPose.getX(), shooter.currentRobotPose.getY(), Math.toDegrees(shooter.currentRobotPose.getHeading()));
        telem.addData(TelemetryMode.RAW_DATA, "REV 9-axis IMU heading", shooter.rev9AxisImuHeadingDeg());
        telem.addData(TelemetryMode.INFO, "future robot pose", "x: %.2f, y: %.2f, heading: %.2f", shooter.futureRobotPose.getX(), shooter.futureRobotPose.getY(), Math.toDegrees(shooter.futureRobotPose.getHeading()));
        telem.addData(TelemetryMode.INFO, "is turret looking ahead", shooter.isTurretLookingAhead());
        telem.addData(TelemetryMode.RAW_DATA, "turret lookahead time", shooter.getTurretTimeLookahead());

        telem.addData(TelemetryMode.RAW_DATA, "Lift Engaged", intake.getLiftEngaged());
        telem.addData(TelemetryMode.RAW_DATA, "Lift Position", intake.getLiftPosition());

        telem.addData(TelemetryMode.RAW_DATA, "f p", "%.5f", shooter.flywheel.p);
        telem.addData(TelemetryMode.RAW_DATA, "f i", "%.5f", shooter.flywheel.i);
        telem.addData(TelemetryMode.RAW_DATA, "f d", "%.5f", shooter.flywheel.d);
        telem.addData(TelemetryMode.RAW_DATA, "f v", "%.5f", shooter.flywheel.v);
        telem.addData(TelemetryMode.RAW_DATA, "flywheel power", shooter.flywheel.getMotorPowers()[0]);

        telem.addData(TelemetryMode.RAW_DATA, "turret current position", shooter.turret.getCurrentPosition());
        telem.addData(TelemetryMode.RAW_DATA, "turret target position", shooter.turret.getTargetPosition());

        telem.addData(TelemetryMode.RAW_DATA, "t p", "%.5f", shooter.turret.p);
        telem.addData(TelemetryMode.RAW_DATA, "t i", "%.5f", shooter.turret.i);
        telem.addData(TelemetryMode.RAW_DATA, "t d", "%.5f", shooter.turret.d);
        telem.addData(TelemetryMode.RAW_DATA, "t f", "%.5f", shooter.turret.f);
        telem.addData(TelemetryMode.RAW_DATA, "turret power", shooter.turret.getServoPowers()[0]);

        telem.addData(TelemetryMode.RAW_DATA, "pedro follower busy?", pedroDrive.follower.isBusy());
        telem.addData(TelemetryMode.RAW_DATA, "pedro t value", "%.2f", pedroDrive.follower.getCurrentTValue());

        telem.update();
    }
}
