package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import javax.annotation.Nullable;

/// USES EXTERNAL ENCODER
@Peak
public final class ExtremePrecisionFlywheel {

    private final Encoder encoder;

    public Encoder getEncoder() {
        return encoder;
    }

    private final DcMotorEx leftFlywheel; //has encoder
    private final DcMotorEx rightFlywheel; //follows leftFlywheel

    private VoltageSensor batteryVoltageSensor;

    public ExtremePrecisionFlywheel(DcMotorEx leftFlywheel, DcMotorEx rightFlywheel) {

        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;

        this.leftFlywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[0]);
        this.rightFlywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[1]);

        encoder = new Encoder(leftFlywheel);
        encoder.setDirection(Encoder.Direction.REVERSE);
        encoder.initializeVelocityKalmanFilter(
                Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[0],
                Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[1],
                Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[2],
                Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[3]
        );

        this.leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void initVoltageSensor(HardwareMap hardwareMap) {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public double kp;
    public double kiFar, kiClose;
    private double ki;
    public double getRealKi() {
        return ki;
    }
    public double kISmash;
    private double kISwitchError;
    private double kISwitchTargetVelocity;
    public double getKISwitchTargetVelocity() {
        return kISwitchTargetVelocity;
    }
    public double kd;
    public double kf;
    public double unscaledKv;
    public double kv;
    public double ka;
    public double ks;

    private double kPIDFUnitsPerVolt;

    private double VbackEMF;

    private double MOTOR_RPM;

    private double FN;

    private double SHAFT_RADIUS;

    private double targetVelocity;
    private double currentFilteredVelocity;

    private double lastCurrentFilteredVelocity = 0;

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

    public double p = 0, i = 0, d = 0;
    private double errorSum = 0;
    public double f = 0; //constant feedforward - can be enabled or disabled
    public double v = 0, a = 0;
    public double s = 0;

    public double i_max = Double.MAX_VALUE;
    public double i_min = -Double.MAX_VALUE;

    public void setIConstraints(double i_min, double i_max) {

        this.i_min = i_min;
        this.i_max = i_max;
    }

    public double p_max = Double.MAX_VALUE;
    public double p_min = -Double.MAX_VALUE;

    public void setPConstraints(double p_min, double p_max) {

        this.p_min = p_min;
        this.p_max = p_max;
    }

    public double filteredVoltage;

    /// The alpha that determines the filtering done by the Low Pass filter. Default is 1 (no filtering)
    private double voltageFilterAlpha = 1;

    public void setVoltageFilterAlpha(double voltageFilterAlpha) {
        this.voltageFilterAlpha = voltageFilterAlpha;
    }

    public void updateKvBasedOnVoltage() {

        filteredVoltage = LowPassFilter.getFilteredValue(filteredVoltage, batteryVoltageSensor.getVoltage(), voltageFilterAlpha);

        kv = ShooterInformation.Models.getScaledFlywheelKv(unscaledKv, filteredVoltage);
    }


    public double[] getPIDFVAS() {

        return new double[] {p, i, d, f, v, a, s};
    }

    /// @param kp Proportional
    /// <p>
    /// @param kiFar Integral
    /// @param kiClose Integral
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
    /// <p>
    /// @param kISmash Multiplies I to decrease I when error switches
    /// <p>
    /// @param kISwitchError amount at error that the error must be less then to switch to using kiClose from kiFar
    public void setVelocityPIDFVASCoefficients(double kp, double kiFar, double kiClose, double kd, double kf, double kv, double ka, double ks, double kPIDFUnitsPerVolt, double kISmash, double kISwitchError) {

        this.kp = kp;
        this.kiFar = kiFar;
        this.kiClose = kiClose;
        this.kd = kd;
        this.kf = kf;
        this.kv = unscaledKv = kv;
        this.ka = ka;
        this.ks = ks;
        this.kPIDFUnitsPerVolt = kPIDFUnitsPerVolt;
        this.kISmash = kISmash;
        this.kISwitchError = kISwitchError;
    }

    private double currentTime = 0;

    private double prevTime = 0, prevError = 0;

    /// @param velocity in ticks per second
    public void setVelocity(double velocity, boolean allowIntegralReset) {

        if (allowIntegralReset && targetVelocity != velocity) {

            resetIntegral(); //resetting integral when target velocity changes to prevent integral windup
        }

        burstVelocity = 0;

        targetVelocity = velocity;
    }

    /// @param velocity in ticks per second
    /// @param burst initial burst velocity in ticks per second added to 'velocity'
    public void setVelocityWithBurst(double velocity, @Nullable Double burst, boolean allowIntegralReset) {

        if (allowIntegralReset && targetVelocity != velocity) {
            resetIntegral(); //resetting integral when target velocity changes to prevent integral windup
        }

        if (burst != null) burstVelocity = burst;
        else burstVelocity = 0;

        targetVelocity = velocity;

        targetVelocity += burstVelocity;
    }

    private double velocityEstimate = 0;

    private boolean firstTick = true;
    private double startTime;

    private double seconds() {
        return System.nanoTime() * 1e-9;
    }

    private double power = 0;

    public void update(/*, Telemetry telemetry*/) {

        //setting start time
        if (firstTick) {

            startTime = seconds();
            firstTick = false;
        }

        prevTime = currentTime;
        currentTime = seconds() - startTime;
        double dt = currentTime - prevTime;

        burstVelocity = Math.max(0, burstVelocity - (BURST_DECELERATION_RATE * dt));

        lastCurrentFilteredVelocity = currentFilteredVelocity;

        lastPosition = currentPosition;
        currentPosition = encoder.getCurrentPosition();

        velocityEstimate = (currentPosition - lastPosition) / dt;

        encoder.runVelocityCalculation(velocityEstimate);
        currentFilteredVelocity = encoder.getRealVelocity();
        //currentFilteredVelocity = velocityEstimate;

        //telemetry.addData("current vel", currentVelocity);

        double error = targetVelocity - currentFilteredVelocity;


        //proportional
        p = kp * error;
        p = MathUtil.clamp(p, p_min, p_max);

        //integral - is in fact reset when target velocity changes IF ALLOWED
        if (kISwitchTargetVelocity == targetVelocity || error < kISwitchError) {

            ki = kiClose;
            resetIntegral();
            kISwitchTargetVelocity = targetVelocity;
        }
        else {
            ki = kiFar;
        }

        if (!Double.isNaN(error * dt) && error * dt != 0 && targetVelocity != 0) errorSum += error * dt;
        else errorSum = 0; //integral is reset if it's NaN or if targetVelocity is equal to 0
        // i is prevented from getting too high or too low
        i = MathUtil.clamp(ki * errorSum, i_min, i_max);

        // i smashing
        if (Math.signum(error) != Math.signum(prevError)) {
            errorSum *= kISmash;
        }

        //derivative
        d = dt > 0 ? kd * (error - prevError) / dt : 0;
        if (!MathUtil.valueWithinRangeIncludingPoles(d, -1, 1)) d = 0;
        d = MathUtil.clamp(d, -0.2, 0.2);

        //positional feedforward for holding
        f = targetVelocity != 0 ? kf : 0;

        //velocity feedforward
        v = kv * targetVelocity;

        targetAcceleration = -targetVelocity / dt;
        a = ka * targetAcceleration;

        //static friction
        double freeSpeed = (MOTOR_RPM * PI) / 30; // in rad/s
        double ke = VbackEMF / freeSpeed; // using ke instead of kt - #1 ks will compensate, #2 ke can more easily be calculate accurately
        double T = ks * FN * SHAFT_RADIUS;
        s = targetVelocity != 0 ? (T / ke) * kPIDFUnitsPerVolt : 0;

        double PIDFVAPower = p + i + d + (usingHoldingFeedforward ? f : 0) + v + a;

        //telemetry.addData("PIDFVAPower", PIDFVAPower);

        power = PIDFVAPower + (Math.signum(PIDFVAPower) >= 0 ? s : -s);

        if (targetVelocity == 0) power = 0;

        //telemetry.addData("power", power);

        if (isMotorEnabled) {
            leftFlywheel.setPower(power);
            rightFlywheel.setPower(power);
        }

        prevError = error;
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

    public void setPower(double power) {

        if (isMotorEnabled) throw new IllegalArgumentException("Must disable PID mode!");

        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);
    }

    // default mode is enabled
    private boolean isMotorEnabled = true;

    /// @return if motor is enabled
    public boolean getMotorEnabled() {
        return isMotorEnabled;
    }

    public void runMotor(RunningMotor isMotorEnabled) {
        this.isMotorEnabled = isMotorEnabled.getValue();
    }

    /// IN TICKS PER SECOND
    /// <p>
    ///Calculated by external encoder using ticks
    /// <p>
    ///Gets past encoder overflow, if you're using a high resolution encoder like the REV Through-Bore, it's prone to these problems.
    /// <p>
    /// Uses Kalman Filter
    public double getRealVelocity() {
        return currentFilteredVelocity;
    }

    /// @return distance in ticks / time in seconds => ticks per second
    public double getCurrentVelocityEstimate() {
        return velocityEstimate;
    }

    public double getLastRealVelocity() {
        return lastCurrentFilteredVelocity;
    }

    public double getBurstVelocity() {
        return burstVelocity;
    }

    public double getTargetAcceleration() {
        return targetAcceleration;
    }

    public double getTargetVelocity() {
        return targetVelocity;
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
        currentVelocity = Math.abs(this.currentFilteredVelocity);

        if (Math.abs(Math.abs(targetVelocity - lastCurrentFilteredVelocity) - currentVelocity) <= velocityMarginOfError && Math.abs(currentVelocity - lastCurrentFilteredVelocity) < stabilityMarginOfError) motorIsAtVelocityAndStable = true;

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

        currentTime = 0;

        prevError = 0;
        prevTime = 0;

        targetVelocity = 0;
        currentFilteredVelocity = 0;

        lastCurrentFilteredVelocity = 0;

        lastPosition = 0;
        currentPosition = 0;

        setVelocity(0, false); //allowIntegralReset is false to speed up computation because of how '||' works - probably negligible
        resetIntegral(); //integral reset
    }

    private void resetIntegral() {
        errorSum = 0;
        i = 0;
    }

    public double[] getMotorPowers() {
        return new double[] {leftFlywheel.getPower(), rightFlywheel.getPower()};
    }

}