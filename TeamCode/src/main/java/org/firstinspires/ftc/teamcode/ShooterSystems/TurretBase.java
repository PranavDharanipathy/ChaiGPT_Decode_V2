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

import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_DERIVATIVE_POSITION_GAPS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KDS_LEFT;

import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_TARGET_FEEDFORWARD_POSITIONS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KFS;

import java.util.Collections;

/// USES EXTERNAL ENCODER
@Peak
public class TurretBase {

    private final CRServoImplEx leftTurretBase, rightTurretBase;
    private final Encoder encoder;

    public double kp, kiFar, kiClose, kd, ks, kISmash, kDFilter, kPowerFilter;
    public double ki, kf;

    private double maxI = 1;
    private double minI = -1;

    public double dActivation = 0;

    private double iSwitch;

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

    private double fDirection = 1;

    private boolean reversed = false;

    /// Call after setting PIDFS coefficients
    public void reverse() {

        DcMotorSimple.Direction direction = leftTurretBase.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;

        leftTurretBase.setDirection(direction);
        rightTurretBase.setDirection(direction);

        TURRET_DERIVATIVE_POSITION_GAPS.replaceAll(i -> -i);
        TURRET_TARGET_FEEDFORWARD_POSITIONS.replaceAll(i -> -i);

        Collections.reverse(TURRET_DERIVATIVE_POSITION_GAPS);
        Collections.reverse(TURRET_TARGET_FEEDFORWARD_POSITIONS);

        Collections.reverse(TURRET_KDS_LEFT);
        Collections.reverse(TURRET_KFS);

        reversed = true;

        fDirection = -1;

    }

    private TurretBasePIDFSCoefficients coefficients;

    public void setPIDFSCoefficients(TurretBasePIDFSCoefficients coefficients) {

        this.coefficients = coefficients;

        //setting variables that do not change
        ks = coefficients.ks;

        kPowerFilter = coefficients.kPowerFilter;

        minI = coefficients.minI;
        maxI = coefficients.maxI;
    }

    /// @param tuning true means that turret's in tuning mode while false means that turret is in normal mode.
    /// If the object isn't initialized, nothing will happen and the method will deal with the error.
    public void setTuning(boolean tuning) {

        try {
            coefficients.setTuning(tuning);
        }
        catch (Exception ignore) {}
    }

    /// Setting variables that do change
    private void chooseCoefficientsInternal(TurretBasePIDFSCoefficients.TurretSide side) {

        kp = coefficients.kp(side);
        kiFar = coefficients.kiFar(side);
        kiClose = coefficients.kiClose(side);
        kd = coefficients.kd(targetPosition, lastTargetPosition, startPosition, reversed);
        kf = coefficients.kf(targetPosition, lastTargetPosition, startPosition, reversed);

        kDFilter = coefficients.kDFilter(side);

        iSwitch = coefficients.iSwitch(side);

        kISmash = coefficients.kISmash(side);

        dActivation = coefficients.dActivation(side);
    }

    public double startPosition;
    private double lastTargetPosition;
    private double targetPosition;

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

        chooseCoefficientsInternal(TurretBasePIDFSCoefficients.TurretSide.getSide(targetPosition, startPosition, reversed));

        //proportional
        p = kp * error;

        //integral
        ki = Math.abs(error) <= iSwitch ? kiClose : kiFar;
        if (dt != 0) i += ki * error * dt;
        if (Math.signum(error) != Math.signum(prevError) && error != 0) i *= kISmash;
        i = MathUtil.clamp(i, minI, maxI);

        //derivative
        double rawDerivative = (error - prevError) / dt;
        filteredDerivative = LowPassFilter.getFilteredValue(filteredDerivative, rawDerivative, kDFilter);
        d = dt > 0 && Math.abs(error) >= dActivation ? kd * filteredDerivative : 0;

        //feedforward
        double reZeroedTargetPosition = targetPosition - startPosition;
        f = kf * fDirection * reZeroedTargetPosition;

        //static friction feedforward
        s = ks * Math.signum(error);

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
        double[] turretFeedforwardTargetPositions = TURRET_TARGET_FEEDFORWARD_POSITIONS.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(turretFeedforwardTargetPositions, reZeroedTargetPosition);

        double targetPosition0 = bounds[0];
        double targetPosition1 = bounds[1];

        double kf0 = TURRET_KFS.get(TURRET_TARGET_FEEDFORWARD_POSITIONS.indexOf(targetPosition0));
        double kf1 = TURRET_KFS.get(TURRET_TARGET_FEEDFORWARD_POSITIONS.indexOf(targetPosition1));

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