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
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;

@Config
@TeleOp (group = "tuning")
public class FlywheelKalmanFilterVelocityEstimationTuner extends OpMode {

    public static double Q = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[0];
    public static double R = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[1];
    public static double OUTLIER_SIGMA = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[2];
    public static double KR_INFLATION = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[3];
    public static double avgDt = Constants.FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS[4];

    public static long LOOP_TIME = 70;

    public static int STAGE = 0;

    public static double FLYWHEEL_POWER = 0;
    public static double FLYWHEEL_POWER_INCREMENT = 0.005;
    private int powerDirection = 1;

    private double power;

    private DcMotorEx leftFlywheel, rightFlywheel;
    private Encoder encoder;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName);

        leftFlywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[0]);
        rightFlywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[1]);

        encoder = new Encoder(leftFlywheel);
        encoder.setDirection(Encoder.Direction.REVERSE);
    }

    private final ArrayList<Double> dtHistory = new ArrayList<>();

    private final int dtHistoryMaxLength = 300;

    private double prevTicks, currTicks;

    @Override
    public void loop() {

        // gets the average dt
        double instanceDt = TickrateChecker.getTimePerTick();

        //calculates the raw velocity using distance/time
        prevTicks = currTicks;
        currTicks = encoder.getCurrentPosition();
        double velocityEstimate = instanceDt > 0 ? (currTicks - prevTicks) / instanceDt : 0;

        dtHistory.add(instanceDt);

        double dtHistoryLength = dtHistory.size();
        if (dtHistoryLength > dtHistoryMaxLength) dtHistory.remove(0);

        double sumOfDts = 0.0;

        for (Double dt : dtHistory) {
            sumOfDts+=dt;
        }

        double calculatedAvgDt = sumOfDts / (dtHistoryLength);

        // updates constants
        encoder.initializeVelocityKalmanFilter(Q, R, OUTLIER_SIGMA, KR_INFLATION, avgDt != 0 ? avgDt : calculatedAvgDt);

        switch (STAGE) {

            case 0:

                power = FLYWHEEL_POWER;
                break;

            case 1:

                power+=(FLYWHEEL_POWER_INCREMENT * powerDirection);
                power = MathUtil.clamp(power,-1,1);

                if (power == 1 || power == -1) powerDirection*=-1;
                break;
        }

        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);

        encoder.runVelocityCalculation(instanceDt, velocityEstimate/*, telemetry*/);

        telemetry.addData("estimated velocity", velocityEstimate);
        telemetry.addData("real velocity", encoder.getRealVelocity());

        telemetry.addData("instanceDt", instanceDt);
        telemetry.addData("calculatedAvgDt", calculatedAvgDt);
        telemetry.update();

        sleep(LOOP_TIME);

    }
}
