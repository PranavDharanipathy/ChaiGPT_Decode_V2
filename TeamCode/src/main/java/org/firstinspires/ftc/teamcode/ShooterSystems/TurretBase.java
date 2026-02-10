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

import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_PD_POSITIONS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KPS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KDS;

import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_FEEDFORWARD_POSITIONS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KFS;

import java.util.Collections;

/// USES EXTERNAL ENCODER
@Peak
public class TurretBase {

    private final CRServoImplEx leftTurretBase, rightTurretBase;
    private final Encoder encoder;

    public double kp, kiFar, kiClose, kd, ks, kISmash, kDFilter, kPowerFilter, lanyardEquilibrium;
    public double ki, kf;

    private double maxI = 1;
    private double minI = -1;

    public double dActivation = 0;

    private double iSwitch, iSwitchTargetPosition = 0;

    public double p, i, d, f, s;

    public double filteredDerivative = 0;

    public double filteredPower = 0;

    public TurretBase(HardwareMap hardwareMap) {

        this (hardwareMap, null); //using encoder's current position
    }

    /// @param turretStartPosition for re-zeroing the turret compensating for the home position not always being at 0
    public TurretBase(HardwareMap hardwareMap, Double turretStartPosition) {

        leftTurretBase = hardwareMap.get(CRServoImplEx.class, Constants.MapSetterConstants.turretBaseLeftServoDeviceName);
        rightTurretBase = hardwareMap.get(CRServoImplEx.class, Constants.MapSetterConstants.turretBaseRightServoDeviceName);

        leftTurretBase.setDirection(Constants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(Constants.TURRET_BASE_DIRECTIONS[1]);

        leftTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));

        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.turretExternalEncoderMotorPairName));
        encoder.setDirection(Encoder.Direction.FORWARD);

        // first targetPosition is the start position
        double tsp = turretStartPosition != null ? turretStartPosition : encoder.getCurrentPosition();
        lastCurrentPosition = currentPosition = lastTargetPosition = targetPosition = startPosition = tsp;
    }

    private double fDirection = 1;

    private boolean reversed = false;

    /// Call after setting PIDFS coefficients
    public void reverse() {

        DcMotorSimple.Direction direction = leftTurretBase.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;

        leftTurretBase.setDirection(direction);
        rightTurretBase.setDirection(direction);

        TURRET_PD_POSITIONS.replaceAll(i -> -i);
        TURRET_FEEDFORWARD_POSITIONS.replaceAll(i -> -i);

        Collections.reverse(TURRET_PD_POSITIONS);
        Collections.reverse(TURRET_FEEDFORWARD_POSITIONS);

        Collections.reverse(TURRET_KPS);
        Collections.reverse(TURRET_KDS);
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

        lanyardEquilibrium = coefficients.lanyardEquilibrium;

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

    public enum PD_INTERPOLATION_MODE {

        NONE, P, D, BOTH;

        /**
         * <p>"00" - Indicates NONE
         * <p>"10" - Indicates P
         * <p>"01" - Indicates D
         * <p>"11" - Indicates BOTH
         * @throws IllegalArgumentException If an invalid mode is inputted
         */
        public static PD_INTERPOLATION_MODE fromString(String mode) {

            switch (mode) {

                case "00":
                    return NONE;

                case "10":
                    return P;

                case "01":
                    return D;

                case "11":
                    return BOTH;

                default:
                    throw new IllegalArgumentException(mode + " is an invalid mode string!");
            }
        }
    }

    public PD_INTERPOLATION_MODE pdInterpolationMode = PD_INTERPOLATION_MODE.BOTH;

    /// To be able to set from FTC Dashboard
    public void setPdInterpolationMode(PD_INTERPOLATION_MODE mode) {
        pdInterpolationMode = mode;
    }

    /// Setting variables that do in fact change
    private void chooseCoefficientsInternal(TurretBasePIDFSCoefficients.TurretSide side) {

        if (pdInterpolationMode.equals(PD_INTERPOLATION_MODE.BOTH)) {

            double[] kpAndKd = coefficients.kpAndKd(targetPosition, startPosition);
            kp = kpAndKd[0];
            kd = kpAndKd[1];
        }
        else if (pdInterpolationMode.equals(PD_INTERPOLATION_MODE.P)) {

            kp = coefficients.kp(targetPosition, startPosition);
            kd = coefficients.kd;
        }
        else if(pdInterpolationMode.equals(PD_INTERPOLATION_MODE.D)) {

            kp = coefficients.kp;
            kd = coefficients.kd(targetPosition, startPosition);
        }
        else { //if pdInterpolationMode equals PD_INTERPOLATION_MODE.NONE

            kp = coefficients.kp;
            kd = coefficients.kd;
        }


        kiFar = coefficients.kiFar(side);
        kiClose = coefficients.kiClose(side);
        kf = coefficients.kf(targetPosition, lastTargetPosition, startPosition, currentPosition, reversed);

        kDFilter = coefficients.kDFilter(side);

        iSwitch = coefficients.iSwitch(side);

        kISmash = coefficients.kISmash(side);

        dActivation = coefficients.dActivation(side);
    }

    public double startPosition;
    private double lastTargetPosition;
    private double targetPosition;

    private double currentPosition;
    private double lastCurrentPosition;

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

    public double getLastCurrentPosition() {
        return lastCurrentPosition;
    }

    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    private double prevError, error;
    private double prevTime, currTime;

    private ElapsedTime timer = new ElapsedTime();

    public void update() {

        lastCurrentPosition = currentPosition;
        currentPosition = getCurrentPosition();

        currTime = timer.milliseconds();
        double dt = currTime - prevTime;

        error = targetPosition - currentPosition;

        chooseCoefficientsInternal(TurretBasePIDFSCoefficients.TurretSide.getSide(targetPosition, startPosition, reversed));

        //proportional
        p = kp * error;

        //integral
        if (targetPosition == iSwitchTargetPosition || Math.abs(error) <= iSwitch) {

            iSwitchTargetPosition = targetPosition;
            ki = kiClose;
        }
        else {
            ki = kiFar;
        }
        if (dt != 0) i += ki * error * dt;
        if (Math.signum(error) != Math.signum(prevError) && error != 0) i *= kISmash;
        i = MathUtil.clamp(i, minI, maxI);

        //derivative
        double rawDerivative = (error - prevError) / dt;
        filteredDerivative = LowPassFilter.getFilteredValue(filteredDerivative, rawDerivative, kDFilter);
        d = dt > 0 && Math.abs(error) >= dActivation ? kd * filteredDerivative : 0;

        //feedforward
        double reZeroedTargetPosition = targetPosition - startPosition;
        f = kf * fDirection * (reZeroedTargetPosition - lanyardEquilibrium);

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

}