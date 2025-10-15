package org.firstinspires.ftc.teamcode.TurretSystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.SimpleMathUtil;

public class TurretBase {

    private final CRServoImplEx leftTurretBase;
    private final CRServoImplEx rightTurretBase;

    private final DcMotor encoder;

    /// @param encoderMotorName Ima just use a random motor for the REV Through-Bore Encoder and get info from that motor cuz it works.
    public TurretBase(HardwareMap hardwareMap, String leftTurretBaseServoName, String rightTurretBaseServoName, String encoderMotorName) {

        leftTurretBase = hardwareMap.get(CRServoImplEx.class, leftTurretBaseServoName);
        rightTurretBase = hardwareMap.get(CRServoImplEx.class, rightTurretBaseServoName);

        encoder = hardwareMap.get(DcMotor.class, encoderMotorName);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTurretBase.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));
        rightTurretBase.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));

        leftTurretBase.setDirection(Constants.TURRET_BASE_DIRECTION);
        rightTurretBase.setDirection(Constants.TURRET_BASE_DIRECTION);
    }

    private ElapsedTime timer;

    private double prevTime, currTime;

    private double targetPosition;

    private double prevError, error;

    private double kp, ki, kd, kf;

    public void setPIDFCoefficients(double kp, double ki, double kd, double kf) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    private double p, i, d, f;

    public void setPosition(double position, double integralResetMargin) {

        if (Math.abs(position) - integralResetMargin > Math.abs(targetPosition) && Math.abs(position) + integralResetMargin < Math.abs(targetPosition)) i = 0;
        targetPosition = position;
    }

    private Double min_i = Double.MIN_VALUE, max_i = Double.MAX_VALUE;

    public void setIConstraints(double min_i, double max_i) {

        this.min_i = min_i;
        this.max_i = max_i;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    public void resetIntegral() { i = 0; }

    public void update() {

        currTime = timer.milliseconds();
        double dt = currTime - prevTime;

        error = targetPosition - encoder.getCurrentPosition();

        p = kp * error;

        i += ki * error / dt;
        i = SimpleMathUtil.clamp(i, min_i, max_i);

        d = kd * (error - prevError) / dt;

        f = kf;

        double power = p + i + d + f;

        leftTurretBase.setPower(power);
        rightTurretBase.setPower(power);

        prevError = error;
        prevTime = currTime;
    }

}
