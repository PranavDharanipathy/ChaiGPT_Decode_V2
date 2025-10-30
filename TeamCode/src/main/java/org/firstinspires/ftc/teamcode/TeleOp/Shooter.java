package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.SubsystemInternal;

public class Shooter implements SubsystemInternal {

    private BetterGamepad controller1, controller2;

    public ExtremePrecisionFlywheel flywheel;

    private TurretBase turret;

    private Limelight3A limelight;

    private ElapsedTime timer = new ElapsedTime();

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
    private double position;

    public void start() {

        startPosition = turret.getCurrentPosition();
        position = startPosition;

        MIN_TURRET_POSITION = ShooterInformation.ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE;
        MAX_TURRET_POSITION = ShooterInformation.ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE;

        flywheel.reset();
        limelight.start();
    }

    private boolean shooterToggle = false;

    public enum FLYWHEEL_VELOCITY {

        FAR_SIDE(ShooterInformation.ShooterConstants.FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY), CLOSE_SIDE(ShooterInformation.ShooterConstants.CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY);

        private double velocity;

        FLYWHEEL_VELOCITY(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    private FLYWHEEL_VELOCITY launchZoneVelocity = FLYWHEEL_VELOCITY.FAR_SIDE;

    private LLResult result;

    public void update() {

        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        if (controller2.dpad_up()) launchZoneVelocity = FLYWHEEL_VELOCITY.CLOSE_SIDE;
        else if (controller2.dpad_down()) launchZoneVelocity = FLYWHEEL_VELOCITY.FAR_SIDE;

        //shooter
        if (shooterToggle) {
            flywheel.setVelocity(launchZoneVelocity.getVelocity(), true);
        }
        else {
            flywheel.setVelocity(0, true);
        }

        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            if (ShooterInformation.Regressions.getDistanceFromRegression(result.getTy()) > 0) turret.setMultiplier(ShooterInformation.ShooterConstants.CLOSE_MULTIPLIER);
            else turret.setMultiplier(ShooterInformation.ShooterConstants.FAR_MULTIPLIER);

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

        if (controller2.right_stick_x() > Constants.JOYSTICK_MINIMUM) {
            position += Constants.TURRET_MANUAL_ADJUSTMENT;
        }
        else if (controller2.right_stick_x() < -Constants.JOYSTICK_MINIMUM) {
            position -= Constants.TURRET_MANUAL_ADJUSTMENT;
        }
        else{
            turret.setPosition(MathUtil.clamp(position, MIN_TURRET_POSITION + startPosition, MAX_TURRET_POSITION + startPosition));
        }

        if (timer.milliseconds() >= Constants.FLYWHEEL_PIDFVAS_LOOP_TIME) {
            //run instance of flywheel and turret systems
            timer.reset();
            flywheel.update();
        }

        turret.update();
    }

    private volatile int pipeline;

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

    public double getAdjustedTx() {
        return ShooterInformation.ShooterConstants.getAdjustedTx(result.getTx(), ShooterInformation.Regressions.getDistanceFromRegression(result.getTy()));
    }
}
