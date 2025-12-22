package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.InterpolationData;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_FEEDFORWARD_TARGET_POSITIONS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KFS;

/// USES EXTERNAL ENCODER
@Peak
public class TurretBase {

    private final CRServoImplEx leftTurretBase, rightTurretBase;
    private final Encoder encoder;

    private double kp, ki, kd, ks, kISmash, kDFilter, kPowerFilter;
    private Double kf;
    private double realKf;

    public double getRealKf() { return realKf; }

    private double lanyardEquilibrium;
    public double errorSum;
    public double p, i, d, f, s;

    public double filteredDerivative = 0;
    public double filteredPower = 0;

    public TurretBase(HardwareMap hardwareMap) {

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
    }

    public void reverse() {

        DcMotorSimple.Direction direction = leftTurretBase.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;

        leftTurretBase.setDirection(direction);
        rightTurretBase.setDirection(direction);
    }

    public void setPIDFCoefficients(Double kp, Double ki, Double kd, Double kf, Double ks, Double kISmash, Double kDFilter, Double kPowerFilter, Double lanyardEquilibrium) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.ks = ks;

        this.kISmash = kISmash;

        this.kDFilter = kDFilter;
        this.kPowerFilter = kPowerFilter;

        this.lanyardEquilibrium = lanyardEquilibrium;
    }

    private double startPosition;
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

    public void update() {

        double currentPosition = getCurrentPosition();

        currTime = timer.milliseconds();
        double dt = currTime - prevTime;

        error = targetPosition - currentPosition;

        //proportional
        p = kp * error;

        //integral
        errorSum += error * dt;
        if (Math.signum(prevError) != Math.signum(error)) errorSum *= kISmash;

        //derivative
        double rawDerivative = (error - prevError) / dt;
        filteredDerivative = LowPassFilter.getFilteredValue(filteredDerivative, rawDerivative, kDFilter);
        d = dt > 0 ? kd * filteredDerivative : 0;

        //feedforward
        double reZeroedTargetPosition = targetPosition + startPosition;

        int movementDirection;
        if (currentPosition >= targetPosition) movementDirection = 1;
        else movementDirection = -1;

        // if the turret moves a certain amount beyond it's target position, the feedforward is shut off until it moves back within a range after which the feedforward is enabled again.
        boolean feedforwardOverride = movementDirection == 1
                ? error <= -ShooterInformation.ShooterConstants.TURRET_HOLD_OVERRIDE
                : error >= ShooterInformation.ShooterConstants.TURRET_HOLD_OVERRIDE;

        setUsingFeedforwardState(!feedforwardOverride);

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

        f = usingFeedforward ? realKf * (reZeroedTargetPosition - lanyardEquilibrium) : 0;

        //static friction feedforward
        s = ks * Math.signum(error);

        i = MathUtil.clamp(ki * errorSum, MIN_I, MAX_I);

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

    private boolean usingFeedforward = true;

    public boolean isUsingFeedforward() {
        return usingFeedforward;
    }

    public void setUsingFeedforwardState(boolean usingFeedforward) {
        this.usingFeedforward = usingFeedforward;
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

    /// used for tuning
    public void updateCoefficients(Double kp, Double ki, Double kd, Double kf, Double ks, Double kISmash, Double kDFilter, Double kPowerFilter, Double lanyardEquilibrium) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.ks = ks;

        this.kISmash = kISmash;

        this.kDFilter = kDFilter;
        this.kPowerFilter = kPowerFilter;

        this.lanyardEquilibrium = lanyardEquilibrium;
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

}