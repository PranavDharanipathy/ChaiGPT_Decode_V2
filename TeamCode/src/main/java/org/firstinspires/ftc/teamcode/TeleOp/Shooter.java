package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.Testing.TurretHorizontalGoalAlignment;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.SubsystemInternal;

public class Shooter implements SubsystemInternal {

    private BetterGamepad controller1, controller2;

    private ExtremePrecisionFlywheel flywheel;

    private TurretBase turret;

    private Limelight3A limelight;

    public void provideComponents(ExtremePrecisionFlywheel flywheel, TurretBase turret, Limelight3A unstartedLimelight, BetterGamepad controller1, BetterGamepad controller2) {

        limelight = unstartedLimelight;

        this.flywheel = flywheel;

        this.turret = turret;

        this.controller1 = controller1;
        this.controller2 = controller2;

    }

    private double MIN_TURRET_POSITION, MAX_TURRET_POSITION;

    private double startPosition;

    private double lastTx = 0;
    private double tx = 0;
    private volatile double position;

    public void start() {

        startPosition = turret.getCurrentPosition();
        position = startPosition;

        MIN_TURRET_POSITION = ShooterInformation.ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE;
        MAX_TURRET_POSITION = ShooterInformation.ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE;

        flywheel.reset();
        limelight.start();
    }

    private boolean shooterToggle = false;

    public void update() {

        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        //shooter
        if (shooterToggle) {
            flywheel.setVelocity(ShooterInformation.ShooterConstants.FLYWHEEL_SHOOT_VELOCITY, true);
        }
        else {
            flywheel.setVelocity(0, true);
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            lastTx = tx;
            tx = result.getTx();
        }

        double currentPosition = turret.getCurrentPosition();

        if (result != null && result.isValid()) {
            position = currentPosition + (tx * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE);
        }
        else {
            position = currentPosition + (lastTx * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE);
        }

        turret.setPosition(MathUtil.clamp(position, MIN_TURRET_POSITION + startPosition, MAX_TURRET_POSITION + startPosition));

        //run instance of flywheel and turret systems
        flywheel.update();
        turret.update();
    }

    private int pipeline;

    public synchronized void setPipeline(int pipeline) {

        if (this.pipeline != pipeline) {
            limelight.pipelineSwitch(pipeline);
            this.pipeline = pipeline;
        }
    }

    public int getSetPipeline() {
        return pipeline;
    }

    public int getCurrentPipeline() {
        return limelight.getLatestResult().getPipelineIndex();
    }
}
