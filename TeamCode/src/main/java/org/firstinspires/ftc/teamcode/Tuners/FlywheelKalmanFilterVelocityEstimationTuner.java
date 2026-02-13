package org.firstinspires.ftc.teamcode.Tuners;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Config
@TeleOp (group = "tuning")
public class FlywheelKalmanFilterVelocityEstimationTuner extends OpMode {

    public static double Q = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[0];
    public static double R = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[1];
    public static double OUTLIER_SIGMA = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[2];
    public static double KR_INFLATION = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[3];

    public static long LOOP_TIME = 70;

    public static int STAGE = 0;

    public static double VELOCITY = 0;
    public static double VELOCITY_INCREMENT = 0.005;
    public static double MAX_VELOCITY = 520_000;
    private int velDirection = 1;

    private double vel;

    private ExtremePrecisionFlywheel flywheel;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = new ExtremePrecisionFlywheel(
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        flywheel.initVoltageSensor(hardwareMap);

        flywheel.setInternalParameters(
                ShooterInformation.ShooterConstants.getTotalFlywheelAssemblyWeight(),
                ShooterInformation.ShooterConstants.SHAFT_DIAMETER,
                ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_RPM
        );

        flywheel.setPConstraints(Constants.FLYWHEEL_MIN_PROPORTIONAL_LIMIT, Constants.FLYWHEEL_MAX_PROPORTIONAL_LIMIT);
        flywheel.setIConstraints(Constants.FLYWHEEL_MIN_INTEGRAL_LIMIT, Constants.FLYWHEEL_MAX_INTEGRAL_LIMIT);
        flywheel.setVoltageFilterAlpha(Constants.FLYWHEEL_VOLTAGE_FILTER_ALPHA);

        flywheel.setVelocityPIDVSCoefficients(
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[0],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[1],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[2],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[3],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[4],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[5],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[6],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[7],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[8]
        );
    }

    @Override
    public void loop() {

        double dt = TickrateChecker.getTimePerTick();

        // updates constants
        flywheel.getEncoder().initializeVelocityKalmanFilter(Q, R, OUTLIER_SIGMA, KR_INFLATION);

        switch (STAGE) {

            case 0:

                vel = VELOCITY;
                break;

            case 1:

                vel +=(VELOCITY_INCREMENT * velDirection);
                vel = MathUtil.clamp(vel, 0, MAX_VELOCITY);

                if (vel == 0 || vel == MAX_VELOCITY) velDirection *=-1;
                break;
        }

        flywheel.setVelocity(vel, STAGE == 0);

        flywheel.updateKvBasedOnVoltage();
        flywheel.update();

        telemetry.addData("estimated velocity", flywheel.getCurrentVelocityEstimate());
        telemetry.addData("real velocity", flywheel.getEncoder().getRealVelocity());

        telemetry.addData("dt", dt);
        telemetry.update();

        sleep(LOOP_TIME);

    }
}
