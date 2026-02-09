package org.firstinspires.ftc.teamcode.TeleOp;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.VMotor;

@Peak
public class IntakeMotor extends VMotor {

    public enum Function {
        INTAKE, LIFT, UNKNOWN
    }

    public Function function = Function.INTAKE;

    public void setFunction(Function functionNew) {

        //'function' is the current Function
        //'functionNew' is the Function that its being changed to

        if (function == Function.LIFT && functionNew == Function.INTAKE) resetPIDFS();

        if (function == Function.INTAKE && functionNew == Function.LIFT) {

            startTime = getSeconds();
            startPosition = internalMotor.getCurrentPosition();
        }

        function = functionNew;
    }

    public Function getFunction() {
        return function;
    }

    public IntakeMotor(HardwareMap hardwareMap) {

        super(hardwareMap, Constants.MapSetterConstants.intakeMotorDeviceName);

        setDirection(DcMotor.Direction.REVERSE);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        startPosition = internalMotor.getCurrentPosition();
    }

    public void setLiftPIDFSCoefficients(double kp, double ki, double kd, double kf, double ks, double kISmash, double kDFilter, double kOutputFilter, double liftTouchesGroundPosition) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.ks = ks;

        this.kISmash = kISmash;

        this.kDFilter = kDFilter;
        this.kOutputFilter = kOutputFilter;

        this.liftTouchesGroundPosition = liftTouchesGroundPosition;
    }

    private double MIN_I = -1, MAX_I = 1;

    public void setLiftIConstraints(double min_i, double max_i) {

        MIN_I = min_i;
        MAX_I = max_i;
    }

    public void setPosition(double targetPosition) {

        this.targetPosition = targetPosition;
        reZeroedTargetPosition = null;
    }

    public void setReZeroedPosition(double targetPosition) {

        this.targetPosition = targetPosition + startPosition;
        reZeroedTargetPosition = targetPosition;
    }

    public void setPower(double power) {
        this.setpower = power;
    }

    private double setpower;
    private Double reZeroedTargetPosition = null;
    private double startPosition;
    private double targetPosition;
    private double filteredVolts = 0;

    public double p, i, d, f, s;
    public double errorSum;
    private double filteredDerivative = 0;

    public double kp, ki, kd, kf, ks, kDFilter, kOutputFilter;
    private double realKf;
    public double kISmash;
    private double liftTouchesGroundPosition;

    private double prevError, error;
    private double startTime, prevTime;
    private boolean firstTick = true;

    public double getRealKf() {
        return realKf;
    }

    private double getSeconds() {
        return System.nanoTime() * 1e-9;
    }

    public void update() {

        if (firstTick) {

            startTime = getSeconds();
            firstTick = false;
        }

        if (function == Function.INTAKE) {
            //POWER
            internalMotor.setPower(setpower);
        }
        else if (function == Function.LIFT) {
            //PIDFS
            double currentPosition = getReZeroedPosition();

            double currTime = getSeconds() - startTime;
            double dt = currTime - prevTime;

            error = targetPosition - currentPosition;

            //proportional
            p = kp * error;

            //integral
            errorSum += error * dt;
            if (Math.signum(prevError) != Math.signum(error)) errorSum *= kISmash;

            //derivative
            double rawDerivative = dt > 0 ? (error - prevError) / dt : 0;
            if (dt != 0) filteredDerivative = LowPassFilter.getFilteredValue(filteredDerivative, rawDerivative, kDFilter);
            d = dt > 0 ? kd * filteredDerivative : 0;

            //feedforward
            if (reZeroedTargetPosition == null) {
                realKf = targetPosition > liftTouchesGroundPosition ? Constants.getLiftKfFromRegression(targetPosition) : kf;
            }
            else {
                realKf = targetPosition > liftTouchesGroundPosition ? Constants.getLiftKfFromRegression(reZeroedTargetPosition) : kf;
            }

            f = realKf * targetPosition;

            //static friction feedforward
            s = ks * Math.signum(error);

            i = MathUtil.clamp(ki * errorSum, MIN_I, MAX_I);

            double rawVolts = p + i + d + f + s;
            filteredVolts = LowPassFilter.getFilteredValue(filteredVolts, rawVolts, kOutputFilter);

            setVolts(filteredVolts);

            prevTime = currTime;
            prevError = error;
        }
        else {
            internalMotor.setPower(0);
        }
    }

    public void resetPIDFS() {

        p = 0;
        errorSum = 0;
        i = 0;
        filteredDerivative = 0;
        d = 0;
        f = 0;
        s = 0;
        filteredVolts = 0;
    }

    public double getPositionalError() {
        return error;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getReZeroedPosition() {
        return internalMotor.getCurrentPosition() - startPosition;
    }

}