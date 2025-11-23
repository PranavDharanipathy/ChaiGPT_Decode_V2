package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class TurretBase {

    private final CRServoImplEx leftTurretBase, rightTurretBase;
    private final Encoder encoder;

    private double kp, ki, kd, kf, kISmash, kDFilter, kPowerFilter;
    public double p, i, d, ff;

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
        directionOfMovement = 1;
    }

    public void reverse() {

        DcMotorSimple.Direction direction = leftTurretBase.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;

        leftTurretBase.setDirection(direction);
        rightTurretBase.setDirection(direction);
    }

    public void setPIDFCoefficients(double kp, double ki, double kd, double kf, double kISmash, double kDFilter, double kPowerFilter) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        this.kISmash = kISmash;

        this.kDFilter = kDFilter;
        this.kPowerFilter = kPowerFilter;
    }

    private double startPosition;
    private double lastTargetPosition;
    private double targetPosition;

    private double directionOfMovement;

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

            directionOfMovement = targetPosition >= lastTargetPosition ? 1 : -1;
        }
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
        i += ki * error * dt;
        i = MathUtil.clamp(i, MIN_I, MAX_I);
        if (Math.signum(prevError) != Math.signum(error)) i *= kISmash;

        //derivative
        double rawDerivative = (error - prevError) / dt;
        filteredDerivative = LowPassFilter.getFilteredValue(filteredDerivative, rawDerivative, kDFilter);
        d = dt > 0 ? kd * filteredDerivative : 0;

        //feedforward
        double reZeroedTargetPosition = targetPosition + startPosition;
        ff = usingFeedforward ? kf * directionOfMovement * Math.cos(Math.toRadians(reZeroedTargetPosition / ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE)) : 0;

        double rawPower = p + i + d + ff;
        filteredPower = LowPassFilter.getFilteredValue(filteredPower, rawPower, kPowerFilter);

        leftTurretBase.setPower(filteredPower);
        rightTurretBase.setPower(filteredPower);

        prevTime = currTime;
        prevError = error;
    }

    private boolean usingFeedforward = true;

    public void setUsingFeedforwardState(boolean usingFeedforward) {
        this.usingFeedforward = usingFeedforward;
    }

    public double getPositionError() {
        return Math.abs(error);
    }

    public double $getRawPositionError() {
        return error;
    }

    public double[] $getServoPowers() {
        return new double[] {leftTurretBase.getPower(), rightTurretBase.getPower()};
    }

    public void $stopTurret() {
        leftTurretBase.setPower(0);
        rightTurretBase.setPower(0);
    }

    /// used for tuning
    public void updateCoefficients(double kp, double ki, double kd, double kf, double kISmash, double kDFilter, double kPowerFilter) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        this.kISmash = kISmash;

        this.kDFilter = kDFilter;
        this.kPowerFilter = kPowerFilter;
    }

}