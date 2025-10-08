package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/// USES EXTERNAL ENCODER - REV Through Bore Encoder is highly recommended.
/// <p>|<p>
/// PIDFVAS measured by external encoder.
/// <p>P: Proportional
/// <p>I: Integral
/// <p>D: Derivative
/// <p>F: Holding Feedforward
/// <p>V: Velocity Feedforward
/// <p>A: Acceleration Feedforward
/// <p>S: Static Friction
public final class ExtremePrecisionVeloMotor {

    private DcMotorEx internalMotor;

    public ExtremePrecisionVeloMotor(HardwareMap hardwareMap, String deviceName) {

        internalMotor = hardwareMap.get(DcMotorEx.class, deviceName);

        internalMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        internalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDirection(DcMotorEx.RunMode direction) {
        internalMotor.setMode(direction);
    }

    public double kp;
    public double ki;
    public double kd;
    public double kf;
    public double kv;
    public double ka;
    public double ks;

    private double kPIDFUnitsPerVolt;

    private double VbackEMF;

    private double MOTOR_RPM;

    private double FN;

    private double SHAFT_RADIUS;

    private double lastTargetVelocity = 0; //starts at 0
    private double targetVelocity;
    private double currentVelocity;

    private double lastCurrentVelocity = 0;

    private double targetAcceleration;

    private long currentPosition = 0;
    private long lastPosition;

    private double EXTERNAL_ENCODER_RESOLUTION;

    private final double PI = StrictMath.PI;

    /// @param EXTERNAL_ENCODER_RESOLUTION Is the encoder ticks needed by the external encoder for it to complete a full 360 degree turn,
    ///     it may be listed on the website that it was bought from.
    /// @param MASS_IN_GRAMS Is the amount of mass in grams that is connected to the motor.
    /// @param SHAFT_DIAMETER Is the diameter of shaft connecting to motor.
    /// @param MOTOR_CORE_VOLTAGE Check the website you go the motor from, it may tell you what volt motor core the motor has.
    /// @param MOTOR_RPM Is the RPM of the motor.
    public void setInternalParameters(double EXTERNAL_ENCODER_RESOLUTION, double MASS_IN_GRAMS, double SHAFT_DIAMETER, double MOTOR_CORE_VOLTAGE, double MOTOR_RPM) {

        this.EXTERNAL_ENCODER_RESOLUTION = EXTERNAL_ENCODER_RESOLUTION;

        this.MOTOR_RPM = MOTOR_RPM;

        VbackEMF = MOTOR_CORE_VOLTAGE;

        this.SHAFT_RADIUS = SHAFT_DIAMETER / 2;
        FN = /*gravity*/ 9.80665 * (/*converted mass in g to kg*/ MASS_IN_GRAMS * 1000);
    }

    // p i d f v a s
    public double p, i, d;
    public double f; //constant feedforward - can be enabled or disabled
    public double v, a;
    public double s;

    public Double i_max = Double.POSITIVE_INFINITY;
    public Double i_min = Double.NEGATIVE_INFINITY;

    public void setIConstraints(Double i_max, Double i_min) {

        this.i_max = i_max;
        this.i_min = i_min;
    }

    public double[] getPIDFVAS() {

        return new double[] {p, i, d, f, v, a, s};
    }

    /// @param kp Proportional
    /// <p>
    /// @param ki Integral
    /// <p>
    /// <p>
    /// @param kd Derivative
    /// <p>
    /// @param kf Holding Feedforward
    /// <p>
    /// @param kv Velocity Feedforward
    /// <p>
    /// @param ka Acceleration Feedforward
    /// <p>
    /// @param ks Static Friction
    /// <p>
    /// @param kPIDFUnitsPerVolt delta PIDF / delta volts
    public void setVelocityPIDFVASCoefficients(double kp, double ki, double kd, double kf, double kv, double ka, double ks, double kPIDFUnitsPerVolt) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.kv = kv;
        this.ka = ka;
        this.ks = ks;
        this.kPIDFUnitsPerVolt = kPIDFUnitsPerVolt;
    }

    private double prevTime = 0, prevError = 0;

    /// @param velocity in ticks per second
    public void setVelocity(double velocity, boolean allowIntegralReset) {

        if (allowIntegralReset && targetVelocity != velocity) {
            i = 0; //resetting integral when target velocity changes to prevent integral windup
        }

        lastTargetVelocity = targetVelocity;
        targetVelocity = velocity;
    }

    private double startTime;
    private boolean isSettingStartTime = true;

    public void update() {

        //setting start time
        if (isSettingStartTime) {

            startTime = System.nanoTime();

            lastPosition = currentPosition;
            currentPosition = internalMotor.getCurrentPosition();

            isSettingStartTime = false;
        }

        double elapsedTime = System.nanoTime() - startTime;
        double dt = elapsedTime - prevTime;

        double error;

        lastCurrentVelocity = currentVelocity;

        lastPosition = currentPosition;
        currentPosition = internalMotor.getCurrentPosition();

        //setting current velocity in ticks per second (converting from nanosecond)
        long deltaTicks = currentPosition - lastPosition;
        currentVelocity = 1_000_000_000.0 * (deltaTicks / dt);

        error = targetVelocity - currentVelocity;

        //proportional
        p = kp * error;

        //integral - is in fact reset when target velocity changes IF ALLOWED
        i += error * dt;
        // i is prevented from getting too high or too low
        if (Math.abs(i) > i_max) i = i_max;
        else if (Math.abs(i) < i_min) i = i_min;

        //derivative
        d = kd * (error - prevError) / dt;

        //positional feedforward for holding
        f = kf /* *cos(0 degrees) = 1 */;

        //velocity feedforward
        v = kv * targetVelocity;

        //acceleration feedforward - in ticks per millisecond
        targetAcceleration = 1_000_000.0 * (targetVelocity - lastTargetVelocity) / dt;
        a = ka * targetAcceleration;

        //static friction
        double freeSpeed = (MOTOR_RPM * PI) / 30; // in rad/s
        double ke = VbackEMF / freeSpeed; // using ke instead of kt - #1 ks will compensate, #2 ke can more easily be calculate accurately
        double T = ks * FN * SHAFT_RADIUS;
        s = (T / ke) * kPIDFUnitsPerVolt;

        double PIDFVAPower = p + i + d + (usingHoldingFeedforward ? f : 0) + v + a;
        if (isMotorEnabled) internalMotor.setPower(PIDFVAPower + (s * Math.signum(PIDFVAPower)));

        prevError = error;
        prevTime = elapsedTime;
    }

    public enum RunningMotor {

        DISABLE(false), ENABLE(true);

        boolean value;

        RunningMotor(boolean enableOrDisable) {
            value = enableOrDisable;
        }

        public boolean getValue() {
            return value;
        }
    }

    private boolean isMotorEnabled = true;

    public void runMotor(RunningMotor isMotorEnabled) {
        this.isMotorEnabled = isMotorEnabled.getValue();
    }

    /// IN TICKS PER SECOND
    /// <p>
    ///Calculated by external encoder using ticks
    /// <p>
    ///Gets past encoder overflow, if you're using a high resolution encoder like the REV Through-Bore and experiencing overflow, use this.
    public double getFrontendCalculatedVelocity() {
        return currentVelocity;
    }

    public double getTargetAcceleration() {
        return targetAcceleration;
    }

    /// CONDITIONS:
    /// <p>
    /// Is at velocity within a certain margin of error.
    /// <p>
    /// Is stable within a certain margin of error.
    /// @param velocityMarginOfError Acceptable variation in velocity.
    /// @param stabilityMarginOfError Acceptable variation in stability.
    public boolean isAtVelocityAndStable(double velocityMarginOfError, double stabilityMarginOfError) {

        boolean motorIsAtVelocityAndStable = false;

        double currentVelocity; //different for each type of calculation
        currentVelocity = Math.abs(this.currentVelocity);

        if (Math.abs(targetVelocity) - currentVelocity <= velocityMarginOfError && currentVelocity - Math.abs(lastCurrentVelocity) < stabilityMarginOfError) motorIsAtVelocityAndStable = true;

        return motorIsAtVelocityAndStable;
    }

    private boolean usingHoldingFeedforward = true;

    /// Enables/disables holding feedforward
    /// @param state true (using) or false (not using)
    public void setHoldingFeedforwardState(boolean state) {
        usingHoldingFeedforward = state;
    }

    public void reset() {

        startTime = System.nanoTime();

        prevError = 0;
        prevTime = 0;

        targetVelocity = 0;
        currentVelocity = 0;

        lastCurrentVelocity = 0;
        lastTargetVelocity = 0;

        lastPosition = 0;
        currentPosition = 0;

        internalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setVelocity(0, false); //allowIntegralReset is false to speed up computation
        i = 0; //integral reset
    }

}