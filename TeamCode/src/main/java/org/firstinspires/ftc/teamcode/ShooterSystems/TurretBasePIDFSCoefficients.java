package org.firstinspires.ftc.teamcode.ShooterSystems;

import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KDS_LEFT;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KDS_RIGHT;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_KFS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_DERIVATIVE_POSITION_GAPS;
import static org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation.ShooterConstants.TURRET_TARGET_FEEDFORWARD_POSITIONS;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.util.InterpolationData;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.List;

/// Easier usage of the coefficients for the left and right sides of the robot.
public class TurretBasePIDFSCoefficients {

    public double lkp, rkp;
    public double lkiFar, rkiFar;
    public double lkiClose, rkiClose;
    public double kd;
    public Double kf;
    public double ks;

    public double lISwitch, rISwitch;

    public double lkISmash, rkISmash;

    public double lDActivation, rDActivation;
    public double lkDFilter, rkDFilter;

    public double kPowerFilter;

    public double kFResistance;

    public double minI, maxI;

    private boolean tuning = false;

    /// @param tuning true means that turret's in tuning mode while false means that turret is in normal mode.
    public void setTuning(boolean tuning) {
        this.tuning = tuning;
    }

    /// Index 0: left
    /// <p>
    /// Index 1: right
    public TurretBasePIDFSCoefficients(
            double[] kp,
            double[] kiFar,
            double[] kiClose,
            double kd,
            @Nullable Double kf,
            double ks,
            double[] iSwitch,
            double[] kISmash,
            double[] dActivation,
            double[] kDFilter,
            double kPowerFilter,
            double kFResistance,
            double minI,
            double maxI
    ) {

        lkp = kp[0];
        rkp = kp[1];

        lkiFar = kiFar[0];
        rkiFar = kiFar[1];

        lkiClose = kiClose[0];
        rkiClose = kiClose[1];

        this.kd = kd;

        this.kf = kf;
        this.ks = ks;

        lISwitch = iSwitch[0];
        rISwitch = iSwitch[1];

        lkISmash = kISmash[0];
        rkISmash = kISmash[1];

        lDActivation = dActivation[0];
        rDActivation = dActivation[1];

        lkDFilter = kDFilter[0];
        rkDFilter = kDFilter[1];

        this.kPowerFilter = kPowerFilter;

        this.kFResistance = kFResistance;

        this.minI = minI;
        this.maxI = maxI;
    }

    public enum TurretSide {

        LEFT, RIGHT;

        public static TurretSide getSide(double targetPosition, double startPosition, boolean reversed) {

            boolean sideCondition = reversed ? targetPosition > startPosition : targetPosition < startPosition;

            return sideCondition ? TurretSide.RIGHT : TurretSide.LEFT;
        }
    }

    public double kp(TurretSide side) {
        return side == TurretSide.LEFT ? lkp : rkp;
    }

    public double kiFar(TurretSide side) {
        return side == TurretSide.LEFT ? lkiFar : rkiFar;
    }

    public double kiClose(TurretSide side) {
        return side == TurretSide.LEFT ? lkiClose : rkiClose;
    }

    public double kd(double targetPosition, double lastTargetPosition, double startPosition, boolean reversed) {

        if (tuning) return kd;

        double gap = Math.abs(targetPosition - lastTargetPosition);

        TurretSide side = TurretSide.getSide(targetPosition, startPosition, reversed);

        if (!kfReversalNeeded(targetPosition, lastTargetPosition, startPosition, reversed)) return kd;

        if (MathUtil.valueWithinRangeIncludingPoles(gap, TURRET_DERIVATIVE_POSITION_GAPS.get(0), TURRET_DERIVATIVE_POSITION_GAPS.get(TURRET_DERIVATIVE_POSITION_GAPS.size() - 1))) {
            return getKdFromInterpolation(gap, side);
        }

        if (gap < TURRET_DERIVATIVE_POSITION_GAPS.get(0)) {
            return side == TurretSide.LEFT ? TURRET_KDS_LEFT.get(0) : TURRET_KDS_RIGHT.get(0);
        }

        //re-zeroed target position greater than the largest re-zeroed target position in the list
        return side == TurretSide.LEFT ? TURRET_KDS_LEFT.get(TURRET_KDS_LEFT.size() - 1) : TURRET_KDS_RIGHT.get(TURRET_KDS_RIGHT.size() - 1);
    }

    public double kf(double targetPosition, double lastTargetPosition, double startPosition, boolean reversed) {

        boolean reversalNeeded = kfReversalNeeded(targetPosition, lastTargetPosition, startPosition, reversed);

        double reversingValue = reversalNeeded && Math.abs(targetPosition - lastTargetPosition) >= ShooterInformation.ShooterConstants.TURRET_KF_RESISTANCE_ENGAGE_ERROR ? kFResistance : 1;

        double reZeroedTargetPosition = targetPosition - startPosition;

        if (kf != null) return kf;

        if (MathUtil.valueWithinRangeIncludingPoles(reZeroedTargetPosition, TURRET_TARGET_FEEDFORWARD_POSITIONS.get(0), TURRET_TARGET_FEEDFORWARD_POSITIONS.get(TURRET_TARGET_FEEDFORWARD_POSITIONS.size() - 1))) {
            return reversingValue * getKfFromInterpolation(reZeroedTargetPosition);
        }

        if (reZeroedTargetPosition < TURRET_TARGET_FEEDFORWARD_POSITIONS.get(0)) {
            return reversingValue * TURRET_KFS.get(0);
        }

        //re-zeroed target position greater than the largest re-zeroed target position in the list
        return reversingValue * TURRET_KFS.get(TURRET_KFS.size() - 1);
    }

    public double iSwitch(TurretSide side) {
        return side == TurretSide.LEFT ? lISwitch : rISwitch;
    }

    public double kISmash(TurretSide side) {
        return side == TurretSide.LEFT ? lkISmash : rkISmash;
    }

    public double dActivation(TurretSide side) {
        return side == TurretSide.LEFT ? lDActivation : rDActivation;
    }

    public double kDFilter(TurretSide side) {
        return side == TurretSide.LEFT ? lkDFilter : rkDFilter;
    }

    private boolean kfReversalNeeded(double targetPosition, double lastTargetPosition, double startPosition, boolean reversed) {

        final boolean lastSideCondition = reversed ? lastTargetPosition > startPosition : lastTargetPosition < startPosition;
        final TurretSide lastSide = lastSideCondition ? TurretSide.RIGHT : TurretSide.LEFT;

        final boolean sideCondition = reversed ? targetPosition > startPosition : targetPosition < startPosition;
        final TurretSide side = sideCondition ? TurretSide.RIGHT : TurretSide.LEFT;

        if (lastSide != side) return false;

        if (reversed) {

            if (side == TurretSide.LEFT && targetPosition > lastTargetPosition) { //left side moving right to middle
                return true;
            }
            else if (side == TurretSide.RIGHT && targetPosition < lastTargetPosition) { //right side moving left to middle
                return true;
            }
        }
        else {

            if (side == TurretSide.LEFT && targetPosition < lastTargetPosition) { //left side moving right to middle
                return true;
            }
            else if (side == TurretSide.RIGHT && targetPosition > lastTargetPosition) { //right side moving left to middle
                return true;
            }
        }

        return false;
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

    private double getKdFromInterpolation(double gap, TurretSide side) {

        //converting list to array
        double[] turretDerivativePositionGaps = TURRET_DERIVATIVE_POSITION_GAPS.stream().mapToDouble(Double::doubleValue).toArray();

        //getting bounds of the current target position
        double[] bounds = MathUtil.findBoundingValues(turretDerivativePositionGaps, gap);

        double gap0 = bounds[0];
        double gap1 = bounds[1];

        List<Double> turretKds = side == TurretSide.LEFT ? TURRET_KDS_LEFT : TURRET_KDS_RIGHT;

        double kd0 = turretKds.get(TURRET_DERIVATIVE_POSITION_GAPS.indexOf(gap0));
        double kd1 = turretKds.get(TURRET_DERIVATIVE_POSITION_GAPS.indexOf(gap1));

        //returning kf
        return MathUtil.interpolateLinear(

                gap,

                new InterpolationData(
                        new double[] {gap0, kd0},
                        new double[] {gap1, kd1}
                )
        );

    }
}
