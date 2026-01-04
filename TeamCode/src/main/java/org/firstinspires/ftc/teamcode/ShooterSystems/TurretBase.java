package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.InterpolationData;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_FEEDFORWARD_TARGET_POSITIONS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KFS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KF_VOLTAGES;

import java.util.Collections;

/// USES EXTERNAL ENCODER
@Peak
public class TurretBase {

    private final CRServoImplEx leftTurretBase, rightTurretBase;
    private final Encoder encoder;

    private final VoltageSensor batteryVoltageSensor;

    private double kp, kiOut, kiIn, kd, ks, kISmash, kDFilter, kPowerFilter;
    private double realKi, kISwitchError, kISwitchTargetPosition;
    private Double kf;
    private double realKf;
    private double fAdjuster;
    private double kFDampen;
    private double kVoltageFilter;

    public double getFAdjuster() { return fAdjuster; }

    public double getRealKf() { return realKf; }

    public double getRealKi() { return realKi; }

    private double lanyardEquilibrium;
    public double p, i, d, f, s;

    public double filteredDerivative = 0;
    public double filteredPower = 0;

    public double filteredVoltage;

    public double kfScalingVoltage;

    private void setKfScalingVoltage(double voltage) {
        kfScalingVoltage = MathUtil.truncate(voltage, 100);
    }

    public TurretBase(HardwareMap hardwareMap) {

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftTurretBase = hardwareMap.get(CRServoImplEx.class, Constants.MapSetterConstants.turretBaseLeftServoDeviceName);
        rightTurretBase = hardwareMap.get(CRServoImplEx.class, Constants.MapSetterConstants.turretBaseRightServoDeviceName);

        leftTurretBase.setDirection(Constants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(Constants.TURRET_BASE_DIRECTIONS[1]);

        leftTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));

        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.turretExternalEncoderMotorPairName));
        encoder.setDirection(Encoder.Direction.FORWARD);

        // first targetPosition is the position it starts at
        lastTargetPosition = targetPosition = startPosition = encoder.getCurrentPosition();

        filteredVoltage = batteryVoltageSensor.getVoltage(); //setting start voltage
        setKfScalingVoltage(filteredVoltage);
        scaleKfs(kfScalingVoltage); //scale to starting voltage
    }

    private int movementDirection = 1;

    /// Call after setting PIDFS coefficients
    public void reverse() {

        DcMotorSimple.Direction direction = leftTurretBase.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;

        leftTurretBase.setDirection(direction);
        rightTurretBase.setDirection(direction);

        movementDirection = -1;

        lanyardEquilibrium *= -1;
    }

    public void setPIDFSCoefficients(Double kp, Double kiOut, Double kiIn, Double kd, Double kf, Double ks, Double kISmash, Double kISwitchError, Double kDFilter, Double kPowerFilter, Double lanyardEquilibrium, Double kFDampen, Double kVoltageFilter) {

        this.kp = kp;
        this.kiOut = kiOut;
        this.kiIn = kiIn;
        this.kd = kd;
        this.kf = kf;
        this.ks = ks;

        this.kISmash = kISmash;

        this.kISwitchError = kISwitchError;

        this.kDFilter = kDFilter;
        this.kPowerFilter = kPowerFilter;

        this.lanyardEquilibrium = lanyardEquilibrium;

        this.kFDampen = kFDampen;

        this.kVoltageFilter = kVoltageFilter;
    }

    public double startPosition;
    private double lastTargetPosition;
    private double targetPosition;

    private double MAX_I = Double.MAX_VALUE;
    private double MIN_I = -Double.MAX_VALUE;

    public void setIConstraints(double min_i, double max_i) {

        MIN_I = min_i;
        MAX_I = max_i;
    }

    public void setPosition(double position) {

        if (targetPosition != position) {

            lastTargetPosition = targetPosition;
            targetPosition = position;
        }
    }

    public double getLastTargetPosition() {
        return lastTargetPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    private double prevError, error;
    private double prevTime, currTime;

    private ElapsedTime timer = new ElapsedTime();

    private boolean firstTick = true;

    public void update() {

        if (firstTick) {

            if (movementDirection == 1) {

                // if turret was not reversed (like in TurretBaseTuner), make the list
                Collections.reverse(TURRET_KFS);
                Collections.reverse(TURRET_KF_VOLTAGES);
            }

            firstTick = false;
        }

        double currentPosition = getCurrentPosition();

        currTime = timer.milliseconds();
        double dt = currTime - prevTime;

        error = targetPosition - currentPosition;

        //proportional
        p = kp * error;

        //integral
        if (kISwitchTargetPosition == targetPosition || Math.abs(error) < kISwitchError) {

            kISwitchTargetPosition = targetPosition;

            realKi = integralIn() ? kiIn : kiOut;
        }
        else realKi = 0;

        i += realKi * error * dt;
        if (error != 0 && Math.signum(prevError) != Math.signum(error)) i *= (Math.abs(error) < kISwitchError ? kISmash : 0);

        //derivative
        double rawDerivative = (error - prevError) / dt;
        filteredDerivative = LowPassFilter.getFilteredValue(filteredDerivative, rawDerivative, kDFilter);
        d = dt > 0 ? kd * filteredDerivative : 0;

        //feedforward

        //voltage compensating the feedforward
        filteredVoltage = LowPassFilter.getFilteredValue(filteredVoltage, batteryVoltageSensor.getVoltage(), kVoltageFilter);

        if (MathUtil.deadband(filteredVoltage, kfScalingVoltage, ShooterInformation.ShooterConstants.TURRET_VOLTAGE_SCALING_DEADBAND) != kfScalingVoltage) {

            setKfScalingVoltage(filteredVoltage);
            scaleKfs(kfScalingVoltage);
        }

        double reZeroedTargetPosition = targetPosition + startPosition;

        // if the turret moves a certain amount beyond it's target position, the feedforward is dampened until it moves back within a range.
        if (flipFeedforward()) {
            //fAdjuster = -1 * (Math.abs(error) / ShooterInformation.ShooterConstants.TURRET_FEEDFORWARD_FLIP_ERROR);
            fAdjuster = ShooterInformation.Models.getScaledTurretFlippedFAdjuster(error);
        }
        else if (dampFeedforward()) fAdjuster = kFDampen;
        else fAdjuster = 1; //no damp

        if (kf != null) {
            realKf = kf;
        }
        else if (MathUtil.valueWithinRangeIncludingPoles(reZeroedTargetPosition, TURRET_FEEDFORWARD_TARGET_POSITIONS.get(0), TURRET_FEEDFORWARD_TARGET_POSITIONS.get(TURRET_FEEDFORWARD_TARGET_POSITIONS.size() - 1))) {
            realKf = getKfFromInterpolation(reZeroedTargetPosition);
        }
        else if (reZeroedTargetPosition < TURRET_FEEDFORWARD_TARGET_POSITIONS.get(0)) {
            realKf = TURRET_KFS.get(0);
        }
        else { //re-zeroed target position greater than the largest re-zeroed target position in the list
            realKf = TURRET_KFS.get(TURRET_KFS.size() - 1);
        }

        f = fAdjuster * realKf * (reZeroedTargetPosition - lanyardEquilibrium);

        //static friction feedforward
        s = ks * Math.signum(error);

        i = MathUtil.clamp(i, MIN_I, MAX_I);

        double rawPower = p + i + d + f + s;
        filteredPower = LowPassFilter.getFilteredValue(filteredPower, rawPower, kPowerFilter);

        if (powerOverride != null) {
            leftTurretBase.setPower(powerOverride);
            rightTurretBase.setPower(powerOverride);
        }
        else {
            leftTurretBase.setPower(filteredPower);
            rightTurretBase.setPower(filteredPower);
        }

        prevTime = currTime;
        prevError = error;
    }

    private Double powerOverride = null;

    public void overridePower(Double power) {
        powerOverride = power;
    }

    public double getPositionError() {
        return Math.abs(error);
    }

    public double getRawPositionError() {
        return error;
    }

    public double[] getServoPowers() {
        return new double[] {leftTurretBase.getPower(), rightTurretBase.getPower()};
    }

    /// Sets the power to zero for this instance, if the update function sets power later, that power will be set.
    public void stopTurret() {
        leftTurretBase.setPower(0);
        rightTurretBase.setPower(0);
    }

    private double getKfFromInterpolation(double reZeroedTargetPosition) {

        //converting list to array
        double[] turretFeedforwardTargetPositions = TURRET_FEEDFORWARD_TARGET_POSITIONS.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(turretFeedforwardTargetPositions, reZeroedTargetPosition);

        double targetPosition0 = bounds[0];
        double targetPosition1 = bounds[1];

        double kf0 = TURRET_KFS.get(TURRET_FEEDFORWARD_TARGET_POSITIONS.indexOf(targetPosition0));
        double kf1 = TURRET_KFS.get(TURRET_FEEDFORWARD_TARGET_POSITIONS.indexOf(targetPosition1));

        //returning kf
        return MathUtil.interpolateLinear(

                reZeroedTargetPosition,

                new InterpolationData(
                        new double[] {targetPosition0, kf0},
                        new double[] {targetPosition1, kf1}
                )
        );

    }

    private void scaleKfs(double currentVoltage) {

        //converting lists to arrays
        double[] turretFeedforwardCoefficients = TURRET_KFS.stream().mapToDouble(Double::doubleValue).toArray();
        double[] turretFeedforwardVoltages = TURRET_KF_VOLTAGES.stream().mapToDouble(Double::doubleValue).toArray();

        for (int index = 0; index == TURRET_KFS.size() - 1; index++) {

            TURRET_KFS.set(index, turretFeedforwardCoefficients[index] * (turretFeedforwardVoltages[index] / currentVoltage));
        }
    }

    private boolean dampFeedforward() {

        if (targetPosition >= 0 && error < -ShooterInformation.ShooterConstants.TURRET_HOLD_OVERRIDE_ERROR) return true;
        else if (targetPosition < 0 && error > ShooterInformation.ShooterConstants.TURRET_HOLD_OVERRIDE_ERROR) return true;
        else return false; //if currentPosition equals targetPosition that means zero error
    }

    private boolean flipFeedforward() {

        if (targetPosition >= 0 && error < -ShooterInformation.ShooterConstants.TURRET_FEEDFORWARD_FLIP_ERROR) return true;
        else if (targetPosition < 0 && error > ShooterInformation.ShooterConstants.TURRET_FEEDFORWARD_FLIP_ERROR) return true;
        else return false; //if currentPosition equals targetPosition that means zero error
    }

    private boolean integralIn() {

        if (targetPosition >= 0 && error < 0) return true;
        else if (targetPosition < 0 && error > 0) return true;
        else return false; //if currentPosition equals targetPosition that means zero error
    }

}