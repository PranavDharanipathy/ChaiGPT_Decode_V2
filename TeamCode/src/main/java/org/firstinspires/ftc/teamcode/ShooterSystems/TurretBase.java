package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class TurretBase {

    private CRServoImplEx leftTurretBase, rightTurretBase;
    private Encoder encoder;

    private double kp, ki, kd, kf, kISmash;
    public double p, i, d, ff;

    public TurretBase(HardwareMap hardwareMap) {

        leftTurretBase = hardwareMap.get(CRServoImplEx.class, Constants.MapSetterConstants.turretBaseLeftServoDeviceName);
        rightTurretBase = hardwareMap.get(CRServoImplEx.class, Constants.MapSetterConstants.turretBaseRightServoDeviceName);

        leftTurretBase.setDirection(Constants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(Constants.TURRET_BASE_DIRECTIONS[1]);

        leftTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightTurretBase.setPwmRange(new PwmControl.PwmRange(500, 2500));

        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.turretExternalEncoderMotorPairName));
        encoder.setDirection(Encoder.Direction.FORWARD);

    }

    public void setPIDFCoefficients(double kp, double ki, double kd, double kf, double kISmash) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        this.kISmash = kISmash;
    }

    private double targetPosition = 0;

    private double MAX_I = Double.MAX_VALUE;
    private double MIN_I = -Double.MAX_VALUE;

    public void setIConstraints(double min_i, double max_i) {

        MIN_I = min_i;
        MAX_I = max_i;
    }

    public void setPosition(double position) {
        targetPosition = position;
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
        d = kd * (error - prevError) / dt;

        //feedforward
        ff = usingFeedforward ? kf * Math.signum(error) * Math.cos(Math.toRadians(currentPosition / ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE)) : 0;

        double power = p + i + d + ff;

        leftTurretBase.setPower(power);
        rightTurretBase.setPower(power);

        prevTime = currTime;
        prevError = error;
    }

    private boolean usingFeedforward = true;

    public void setUsingFeedforwardState(boolean usingFeedforward) {
        this.usingFeedforward = usingFeedforward;
    }

    public double getPositionError() {
        return Math.abs(targetPosition - getCurrentPosition());
    }

    public double[] $getServoPowers() {
        return new double[] {leftTurretBase.getPower(), rightTurretBase.getPower()};
    }

    public void $stopTurret() {
        leftTurretBase.setPower(0);
        rightTurretBase.setPower(0);
    }

    /// used for tuning
    public void updateCoefficients(double kp, double ki, double kd, double kf, double kISmash) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        this.kISmash = kISmash;
    }

}