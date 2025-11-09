package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.SubsystemInternal;

@Peak
@Config
public class Shooter implements SubsystemInternal {

    public static boolean MANUAL_HOOD_POSITION_FROM_DASH = false;
    public static double HOOD_ANGLER_POSITION = 0;

    private BetterGamepad controller1, controller2;

    public ExtremePrecisionFlywheel flywheel;

    private TurretBase turret;

    public HoodAngler hoodAngler;

    private Limelight3A limelight;

    private ElapsedTime timer = new ElapsedTime();

    public void provideComponents(ExtremePrecisionFlywheel flywheel, TurretBase turret, HoodAngler hoodAngler, Limelight3A unstartedLimelight, BetterGamepad controller1, BetterGamepad controller2) {

        limelight = unstartedLimelight;

        this.flywheel = flywheel;

        this.turret = turret;

        this.hoodAngler = hoodAngler;

        this.controller1 = controller1;
        this.controller2 = controller2;

    }

    private double MIN_TURRET_POSITION, MAX_TURRET_POSITION;

    private double turretStartPosition;

    private double lastTx = 0;
    private double tx = 0;
    private double turretPosition;

    public void start() {

        turretStartPosition = turret.getCurrentPosition();
        turretPosition = turretStartPosition;

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

    private double hoodPosition;

    public LLResult llResult;

    public void update() {

        //flywheel
        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        //incrementing the hood positions
        if (controller2.yHasJustBeenPressed) { //close
            ShooterInformation.ShooterConstants.HOOD_CLOSE_POSITION+=ShooterInformation.ShooterConstants.HOOD_POSITION_MANUAL_INCREMENT;
            manualUpdateHoodPositions();
        }
        else if (controller2.xHasJustBeenPressed) {
            ShooterInformation.ShooterConstants.HOOD_CLOSE_POSITION-=ShooterInformation.ShooterConstants.HOOD_POSITION_MANUAL_INCREMENT;
            manualUpdateHoodPositions();
        }

        if (controller2.bHasJustBeenPressed) { //far
            hoodPosition = ShooterInformation.ShooterConstants.HOOD_FAR_POSITION+=ShooterInformation.ShooterConstants.HOOD_POSITION_MANUAL_INCREMENT;
            manualUpdateHoodPositions();
        }
        else if (controller2.aHasJustBeenPressed) {
            ShooterInformation.ShooterConstants.HOOD_FAR_POSITION-=ShooterInformation.ShooterConstants.HOOD_POSITION_MANUAL_INCREMENT;
            manualUpdateHoodPositions();
        }

        // setting hood and flywheel modes (close or far)
        if (controller1.yHasJustBeenPressed) {

            launchZoneVelocity = FLYWHEEL_VELOCITY.CLOSE_SIDE;
            hoodPosition = ShooterInformation.ShooterConstants.HOOD_CLOSE_POSITION;

            controller2.rumble(ShooterInformation.ShooterConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }
        else if (controller1.bHasJustBeenPressed) {

            launchZoneVelocity = FLYWHEEL_VELOCITY.FAR_SIDE;
            hoodPosition = ShooterInformation.ShooterConstants.HOOD_FAR_POSITION;

            controller2.rumble(ShooterInformation.ShooterConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }

        if (MANUAL_HOOD_POSITION_FROM_DASH) hoodPosition = HOOD_ANGLER_POSITION;

        hoodAngler.setPosition(MathUtil.clamp(hoodPosition, ShooterInformation.ShooterConstants.HOOD_ANGLER_MAX_POSITION, ShooterInformation.ShooterConstants.HOOD_ANGLER_MIN_POSITION));

        if (shooterToggle) {
            flywheel.setVelocity(launchZoneVelocity.getVelocity(), true);
        }
        else {
            flywheel.setVelocity(0, true);
        }

        if (flywheel.getFrontendCalculatedVelocity() > launchZoneVelocity.getVelocity() - ShooterInformation.ShooterConstants.FLYWHEEL_SHOOT_VELOCITY_CONTROLLER_RUMBLE_MARGIN) {
            controller1.rumble(ShooterInformation.ShooterConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
        }
        else {
            controller1.stopRumble();
        }

        //turret
        llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            lastTx = tx;
            tx = llResult.getTx();
        }

        double currentPosition = turret.getCurrentPosition();

        if (llResult != null && llResult.isValid()) {
            turretPosition = currentPosition + (tx * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE);
        }
        else {
            turretPosition = currentPosition + (lastTx * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE);
        }

        if (controller2.right_stick_x() > Constants.JOYSTICK_MINIMUM) {
            turretPosition += Constants.TURRET_MANUAL_ADJUSTMENT;
            lastTx = 0;
        }
        else if (controller2.right_stick_x() < -Constants.JOYSTICK_MINIMUM) {
            turretPosition -= Constants.TURRET_MANUAL_ADJUSTMENT;
            lastTx = 0;
        }
        else{
            turret.setPosition(MathUtil.clamp(turretPosition, MIN_TURRET_POSITION + turretStartPosition, MAX_TURRET_POSITION + turretStartPosition));
        }

        //updating
        if (timer.milliseconds() >= Constants.FLYWHEEL_PIDFVAS_LOOP_TIME) {
            //run instance of flywheel and turret systems
            timer.reset();
            flywheel.update();
        }

        turret.update();
    }

    private void manualUpdateHoodPositions() {

        if (launchZoneVelocity == FLYWHEEL_VELOCITY.CLOSE_SIDE) {
            hoodPosition = ShooterInformation.ShooterConstants.HOOD_CLOSE_POSITION;
        }
        else if (launchZoneVelocity == FLYWHEEL_VELOCITY.FAR_SIDE) {
            hoodPosition = ShooterInformation.ShooterConstants.HOOD_FAR_POSITION;
        }
    }

    private int pipeline;

    public void setPipeline(int pipeline) {

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
