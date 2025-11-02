package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;

@Peak
@Config
@TeleOp (group = "tuning")
public class ExtremePrecisionFlywheelTuner extends LinearOpMode {

    public static int LOOP_TIME = 50;

    public enum TUNING_STAGES {

        PIDFVAS /*1st*/, kPIDFUnitsPerVolt /*2nd*/, BURST_VELOCITY /*3rd*/, STABILITY /*4th*/
    }

    public static TUNING_STAGES TUNING_STAGE = TUNING_STAGES.PIDFVAS;

    // Name of the hub that the motor is on.
    public static String HUB_NAME = "";

    public static double KP = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[0];
    public static double KI = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[1];
    public static double KD = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[2];
    public static double KF = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[3];
    public static double KV = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[4];
    public static double KA = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[5];
    public static double KS = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[6];
    public static double kPIDFUnitsPerVolt = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[7];
    public static double kISmash = Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[8];
    public static double I_MIN = -Double.MAX_VALUE, I_MAX = Double.MAX_VALUE;

    public static double VELOCITY;
    public static double BURST_VELOCITY;

    public static double VELOCITY_MARGIN_OF_ERROR = 50;
    public static double STABILITY_MARGIN_OF_ERROR = 50;

    public static double MASS_IN_GRAMS = ShooterInformation.ShooterConstants.BASE_FLYWHEEL_ASSEMBLY_WEIGHT;
    public static double SHAFT_DIAMETER = ShooterInformation.ShooterConstants.SHAFT_DIAMETER;
    public static double MOTOR_CORE_VOLTAGE = ShooterInformation.ShooterConstants.MOTOR_CORE_VOLTAGE;
    public static double MOTOR_RPM = ShooterInformation.ShooterConstants.MOTOR_RPM;
    public static double BURST_DECELERATION_RATE;

    private ExtremePrecisionFlywheel flywheel;

    private VoltageSensor batteryVoltageSensor;

    private double lastVoltage;
    private double currentVoltage;

    private double lastPIDFUnits = 0;
    private double currentPIDFUnits = 0;

    private DcMotorEx leftFlywheel, rightFlywheel;

    /*
     * TUNING STEPS IN ORDER:
     * Weigh/measure parts to provide setInternalParameters()
     * Set all coefficients to 0 except for kPIDFUnitsPerVolt, set that to 1
     * Tune ks
     * Tune kv - as low as possible
     * Tune ka - as low as possible
     * Tune kf - try to keep it as low AS POSSIBLE, if it's even a bit too high, it can easily mess up your PIDFVAS system
     * Tune kp
     * Tune kd
     * Tune ki - comfortably use integral and lower using kISmash
     * Set kPIDFUnitsPerVolt (kv, ka, and ks should not interfere when getting this)
     * Set everything except kPIDFUnitsPerVolt and ks to 0
     * Update ks - if needed
     * Add the other coefficients back
     * After tuning these, you may or may not want to change your kf - update your kPIDFUnitsPerVolt
     *
     * PLOT THE PIDVA!
     */

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        batteryVoltageSensor = hardwareMap.get(VoltageSensor.class, HUB_NAME);

        leftFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName);

        flywheel = new ExtremePrecisionFlywheel(leftFlywheel, rightFlywheel);
        flywheel.setInternalParameters(MASS_IN_GRAMS, SHAFT_DIAMETER, MOTOR_CORE_VOLTAGE, MOTOR_RPM, BURST_DECELERATION_RATE);

        if (isStopRequested()) return;
        waitForStart();

        flywheel.reset();

        currentVoltage = batteryVoltageSensor.getVoltage();
        lastVoltage = currentVoltage;

        while (opModeIsActive()) {

            flywheel.setIConstraints(I_MIN, I_MAX);

            if (TUNING_STAGE == TUNING_STAGES.kPIDFUnitsPerVolt) flywheel.setVelocityPIDFVASCoefficients(KP, KI, KD, KF, 0, 0, 0, kPIDFUnitsPerVolt, kISmash);
            else flywheel.setVelocityPIDFVASCoefficients(KP, KI, KD, KF, KV, KA, KS, kPIDFUnitsPerVolt, kISmash);

            if (TUNING_STAGE == TUNING_STAGES.BURST_VELOCITY) flywheel.setVelocityWithBurst(VELOCITY, BURST_VELOCITY, true);
            else flywheel.setVelocity(VELOCITY, true);

            flywheel.update(/*, telemetry*/);
            sleep(LOOP_TIME);

            lastVoltage = currentVoltage;
            currentVoltage = batteryVoltageSensor.getVoltage();

            lastPIDFUnits = currentPIDFUnits;
            currentPIDFUnits = flywheel.getPIDFVAS()[0] + flywheel.getPIDFVAS()[1] + flywheel.getPIDFVAS()[2] + flywheel.getPIDFVAS()[3];

            telemetry.addData("Target Velocity", flywheel.getTargetVelocity());
            telemetry.addData("Velocity", flywheel.getFrontendCalculatedVelocity());
            telemetry.addData("p", flywheel.getPIDFVAS()[0]);
            telemetry.addData("i", flywheel.getPIDFVAS()[1]);
            telemetry.addData("d", flywheel.getPIDFVAS()[2]);
            telemetry.addData("f", flywheel.getPIDFVAS()[3]);
            telemetry.addData("v", flywheel.getPIDFVAS()[4]);
            telemetry.addData("a", flywheel.getPIDFVAS()[5]);
            telemetry.addData("s", flywheel.getPIDFVAS()[6]);
            telemetry.addData("Target Acceleration", flywheel.getTargetAcceleration());

            if (TUNING_STAGE == TUNING_STAGES.STABILITY) {
                telemetry.addData("Is At Velocity And Stable", flywheel.isAtVelocityAndStable(VELOCITY_MARGIN_OF_ERROR, STABILITY_MARGIN_OF_ERROR));
            }
            else if (TUNING_STAGE == TUNING_STAGES.BURST_VELOCITY) {
                telemetry.addData("Burst Velocity", flywheel.getBurstVelocity());
            }
            else if (TUNING_STAGE == TUNING_STAGES.kPIDFUnitsPerVolt) {
                telemetry.addData("kPIDFUnitsPerVolt", Math.abs(currentPIDFUnits - lastPIDFUnits) / Math.abs(currentVoltage - lastVoltage));
            }

            telemetry.addData("Left flywheel motor power", flywheel.$getMotorPowers()[0]);
            telemetry.addData("Right flywheel motor power", flywheel.$getMotorPowers()[1]);
            telemetry.addData("Is motor enabled", flywheel.getMotorEnabled());
            telemetry.update();
        }
    }
}