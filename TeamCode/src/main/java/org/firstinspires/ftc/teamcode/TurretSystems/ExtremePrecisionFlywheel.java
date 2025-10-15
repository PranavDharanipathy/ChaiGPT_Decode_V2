package org.firstinspires.ftc.teamcode.TurretSystems;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.SimpleMathUtil;

import javax.annotation.Nullable;

@Peak
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
public final strictfp class ExtremePrecisionFlywheel {

    private final DcMotorEx leftFlyWheel;
    private final DcMotorEx rightFlyWheel;

    public ExtremePrecisionFlywheel(HardwareMap hardwareMap, String leftFlyWheelName, String rightFlyWheelName) {

        leftFlyWheel = hardwareMap.get(DcMotorEx.class, leftFlyWheelName);
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, rightFlyWheelName);

        leftFlyWheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[0]);
        rightFlyWheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[1]);

        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    private double burstVelocity = 0;
    private double BURST_DECELERATION_RATE;

    private double targetAcceleration;

    private long currentPosition = 0;
    private long lastPosition;

    private final double PI = StrictMath.PI;

    /// @param MASS_IN_GRAMS Is the amount of mass in grams that is connected to the motor.
    /// @param SHAFT_DIAMETER Is the diameter of shaft in millimeters connecting to motor.
    /// @param MOTOR_CORE_VOLTAGE Check the website you go the motor from, it may tell you what volt motor core the motor has.
    /// @param MOTOR_RPM Is the RPM of the motor.
    /// @param BURST_DECELERATION_RATE Is the rate at which the burst decelerates.
    public void setInternalParameters(double MASS_IN_GRAMS, double SHAFT_DIAMETER, double MOTOR_CORE_VOLTAGE, double MOTOR_RPM, double BURST_DECELERATION_RATE) {

        this.MOTOR_RPM = MOTOR_RPM;

        VbackEMF = MOTOR_CORE_VOLTAGE;

        this.SHAFT_RADIUS = SHAFT_DIAMETER / 2;
        FN = /*gravity*/ 9.80665 * (/*converted mass in g to kg*/ MASS_IN_GRAMS / 1000);

        this.BURST_DECELERATION_RATE = BURST_DECELERATION_RATE;
    }

    // p i d f v a s
    public double p, i, d;
    public double f; //constant feedforward - can be enabled or disabled
    public double v, a;
    public double s;

    public Double i_max = Double.MAX_VALUE;
    public Double i_min = Double.MIN_VALUE;

    public void setIConstraints(Double i_min, Double i_max) {

        this.i_min = i_min;
        this.i_max = i_max;
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

        burstVelocity = 0;

        lastTargetVelocity = targetVelocity;
        targetVelocity = velocity;
    }

    /// @param velocity in ticks per second
    /// @param burst initial burst velocity in ticks per second added to 'velocity'
    public void setVelocityWithBurst(double velocity, @Nullable Double burst, boolean allowIntegralReset) {

        if (allowIntegralReset && targetVelocity != velocity) {
            i = 0; //resetting integral when target velocity changes to prevent integral windup
        }

        if (burst != null) burstVelocity = burst;
        else burstVelocity = 0;

        lastTargetVelocity = targetVelocity;
        targetVelocity = velocity;

        targetVelocity += burstVelocity;
    }

    private double startTime;
    private boolean isSettingStartTime = true;

    private double power;

    public void update() {

        //setting start time
        if (isSettingStartTime) {

            startTime = System.nanoTime();
            isSettingStartTime = false;
        }

        double elapsedTime = System.nanoTime() - startTime;
        double dt = elapsedTime - prevTime;

        burstVelocity = SimpleMathUtil.clamp(Math.abs(burstVelocity-=BURST_DECELERATION_RATE),0, Double.MAX_VALUE);

        lastCurrentVelocity = currentVelocity;

        lastPosition = currentPosition;
        currentPosition = leftFlyWheel.getCurrentPosition();

        //setting current velocity in ticks per second (converting from nanosecond)
        long deltaTicks = currentPosition - lastPosition;
        currentVelocity = 1_000_000_000.0 * (deltaTicks / dt);

        double error = targetVelocity - currentVelocity;

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
        f = kf /* cos(0 degrees) = 1 so no need to multiply kf by it */;

        //velocity feedforward
        v = kv * VbackEMF / power;

        targetAcceleration = (targetVelocity - lastTargetVelocity) / dt;
        a = ka * targetAcceleration;

        //static friction
        double freeSpeed = (MOTOR_RPM * PI) / 30; // in rad/s
        double ke = VbackEMF / freeSpeed; // using ke instead of kt - #1 ks will compensate, #2 ke can more easily be calculate accurately
        double T = ks * FN * SHAFT_RADIUS;
        s = (T / ke) * kPIDFUnitsPerVolt;

        double PIDFVAPower = p + i + d + (usingHoldingFeedforward ? f : 0) + v + a;

        power = PIDFVAPower + (s * Math.signum(PIDFVAPower));

        if (isMotorEnabled) {
            leftFlyWheel.setPower(power);
            rightFlyWheel.setPower(power);
        }

        prevError = error;
        prevTime = elapsedTime;
    }

    public enum RunningMotor {

        DISABLE(false), ENABLE(true);

        private boolean value;

        RunningMotor(boolean enableOrDisable) {
            value = enableOrDisable;
        }

        public boolean getValue() {
            return value;
        }
    }

    // default mode is enabled
    private boolean isMotorEnabled = true;

    public void runMotor(RunningMotor isMotorEnabled) {
        this.isMotorEnabled = isMotorEnabled.getValue();
    }

    /// IN TICKS PER SECOND
    /// <p>
    ///Calculated by external encoder using ticks
    /// <p>
    ///Gets past encoder overflow, if you're using a high resolution encoder like the REV Through-Bore, it's prone to these problems.
    public double getFrontendCalculatedVelocity() {
        return currentVelocity;
    }

    public double getBurstVelocity() {
        return burstVelocity;
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

    // true by default
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

        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setVelocity(0, false); //allowIntegralReset is false to speed up computation because of how '||' works - probably negligible
        i = 0; //integral reset
    }

    public double[] $getMotorPowers() {
        return new double[] {leftFlyWheel.getPower(), rightFlyWheel.getPower()};
    }

}