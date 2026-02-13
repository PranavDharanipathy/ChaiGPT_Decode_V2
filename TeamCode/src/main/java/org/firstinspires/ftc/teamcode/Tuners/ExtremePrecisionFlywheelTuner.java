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
import org.firstinspires.ftc.teamcode.util.LowPassFilter;

@Peak
@Config
@TeleOp (group = "tuning")
public class ExtremePrecisionFlywheelTuner extends LinearOpMode {

    public static int LOOP_TIME = 60;

    public enum TUNING_STAGES {

        PIDVS /*1st*/, kPIDFUnitsPerVolt/*2nd*/, KV_SCALED /*3rd*/, STABILITY
    }

    public static TUNING_STAGES TUNING_STAGE = TUNING_STAGES.PIDVS;

    public static double KP = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[0];
    public static double KI_FAR = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[1];
    public static double KI_CLOSE = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[2];
    public static double KD = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[3];
    public static double KV = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[4];
    public static double KS = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[5];
    public static double kPIDFUnitsPerVolt = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[6];
    public static double kISmash = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[7];
    public static double kISwitchError = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[8];
    public static double I_MIN = Constants.FLYWHEEL_MIN_INTEGRAL_LIMIT, I_MAX = Constants.FLYWHEEL_MAX_INTEGRAL_LIMIT;
    public static double P_MIN = Constants.FLYWHEEL_MIN_PROPORTIONAL_LIMIT, P_MAX = Constants.FLYWHEEL_MAX_PROPORTIONAL_LIMIT;

    public static double VELOCITY;
    public static double BURST_VELOCITY;

    public static double VELOCITY_MARGIN_OF_ERROR = Constants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR;
    public static double STABILITY_MARGIN_OF_ERROR = Constants.FLYWHEEL_STABILITY_MARGIN_OF_ERROR;

    public static double FLYWHEEL_VOLTAGE_FILTER_ALPHA = Constants.FLYWHEEL_VOLTAGE_FILTER_ALPHA;

    public static double MASS_IN_GRAMS = ShooterInformation.ShooterConstants.BASE_FLYWHEEL_ASSEMBLY_WEIGHT;
    public static double SHAFT_DIAMETER = ShooterInformation.ShooterConstants.SHAFT_DIAMETER;
    public static double MOTOR_CORE_VOLTAGE = ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE;
    public static double MOTOR_RPM = ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_RPM;

    private ExtremePrecisionFlywheel flywheel;

    private VoltageSensor batteryVoltageSensor;

    private double lastVoltage;
    private double currentVoltage;
    private double filteredVoltage;

    private double lastPIDFUnits = 0;
    private double currentPIDFUnits = 0;

    private DcMotorEx leftFlywheel, rightFlywheel;

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName);

        flywheel = new ExtremePrecisionFlywheel(leftFlywheel, rightFlywheel);
        flywheel.setInternalParameters(MASS_IN_GRAMS, SHAFT_DIAMETER, MOTOR_CORE_VOLTAGE, MOTOR_RPM);

        if (isStopRequested()) return;
        waitForStart();

        flywheel.reset();

        currentVoltage = batteryVoltageSensor.getVoltage();
        lastVoltage = currentVoltage;

        while (opModeIsActive()) {

            lastVoltage = currentVoltage;
            currentVoltage = batteryVoltageSensor.getVoltage();

            telemetry.addData("voltage", currentVoltage);

            filteredVoltage = LowPassFilter.getFilteredValue(filteredVoltage, currentVoltage, FLYWHEEL_VOLTAGE_FILTER_ALPHA);

            double kv = TUNING_STAGE == TUNING_STAGES.KV_SCALED ? ShooterInformation.Models.getScaledFlywheelKv(KV, filteredVoltage) : KV;

            flywheel.setIConstraints(I_MIN, I_MAX);
            flywheel.setPConstraints(P_MIN, P_MAX);

            if (TUNING_STAGE == TUNING_STAGES.kPIDFUnitsPerVolt) flywheel.setVelocityPIDVSCoefficients(KP, KI_FAR, KI_CLOSE, KD, 0, 0, kPIDFUnitsPerVolt, kISmash, kISwitchError);
            else flywheel.setVelocityPIDVSCoefficients(KP, KI_FAR, KI_CLOSE, KD, kv, KS, kPIDFUnitsPerVolt, kISmash, kISwitchError);

            flywheel.setVelocity(VELOCITY, true);

            flywheel.update(/*, telemetry*/);
            sleep(LOOP_TIME);

            lastPIDFUnits = currentPIDFUnits;
            currentPIDFUnits = flywheel.getPIDVS()[0] + flywheel.getPIDVS()[1] + flywheel.getPIDVS()[2] + flywheel.getPIDVS()[3];

            telemetry.addData("Target Velocity", flywheel.getTargetVelocity());
            telemetry.addData("Real Velocity", flywheel.getRealVelocity());
            telemetry.addData("Velocity Estimate", flywheel.getCurrentVelocityEstimate());
            telemetry.addData("Real ki", flywheel.getRealKi());
            telemetry.addData("KISwitchTargetVelocity", flywheel.getKISwitchTargetVelocity());
            telemetry.addData("p", flywheel.getPIDVS()[0]);
            telemetry.addData("i", flywheel.getPIDVS()[1]);
            telemetry.addData("d", flywheel.getPIDVS()[2]);
            telemetry.addData("v", flywheel.getPIDVS()[3]);
            telemetry.addData("s", flywheel.getPIDVS()[4]);

            if (TUNING_STAGE == TUNING_STAGES.STABILITY) {
                telemetry.addData("Is At Velocity And Stable", flywheel.isAtVelocityAndStable(VELOCITY_MARGIN_OF_ERROR, STABILITY_MARGIN_OF_ERROR));
            }
            else if (TUNING_STAGE == TUNING_STAGES.kPIDFUnitsPerVolt) {
                telemetry.addData("kPIDFUnitsPerVolt", Math.abs(currentPIDFUnits - lastPIDFUnits) / Math.abs(currentVoltage - lastVoltage));
            }

            telemetry.addData("Left flywheel motor power", flywheel.getMotorPowers()[0]);
            telemetry.addData("Right flywheel motor power", flywheel.getMotorPowers()[1]);
            telemetry.addData("Is motor enabled", flywheel.getMotorEnabled());
            telemetry.update();
        }
    }
}