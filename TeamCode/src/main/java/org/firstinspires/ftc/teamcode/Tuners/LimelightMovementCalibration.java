package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;

@Config
@TeleOp (group = "tuning")
public class LimelightMovementCalibration extends OpMode {

    private Limelight3A limelight;

    private TurretBase turret;

    private Telemetry telemetry;

    private double startPosition;

    public static double SHAKE_DEGREES = 0;
    public static double SHAKE_FREQUENCY = 0;

    private int lastPollHzRate;
    public static int POLL_HZ_RATE = ShooterInformation.CameraConstants.CAMERA_POLL_RATE;

    public enum TUNING_STAGE {
        PIPELINE_STATS, POLL_RATE
    }

    public static TUNING_STAGE tuningStage = TUNING_STAGE.PIPELINE_STATS;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, Constants.MapSetterConstants.limelight3AUSBDeviceName);

        turret = new TurretBase(hardwareMap, Constants.MapSetterConstants.turretBaseLeftServoDeviceName, Constants.MapSetterConstants.turretBaseRightServoDeviceName);
        turret.setPIDFCoefficients(
                Constants.TURRET_PIDF_COEFFICIENTS[0],
                Constants.TURRET_PIDF_COEFFICIENTS[1],
                Constants.TURRET_PIDF_COEFFICIENTS[2],
                Constants.TURRET_PIDF_COEFFICIENTS[3]
        );
        turret.setIConstraints(Constants.TURRET_MIN_INTEGRAL_LIMIT, Constants.TURRET_MAX_INTEGRAL_LIMIT);

        startPosition = turret.getCurrentPosition();

        telemetry.addData("start position", startPosition);
        telemetry.update();
    }

    /// true is right and false is left
    enum SHAKE_TYPE {

        LEFT(false), RIGHT(true);

        private boolean shake;

        SHAKE_TYPE(boolean shake) {
            this.shake = shake;
        }

        public boolean getBoolean() {
            return shake;
        }
    }

    private SHAKE_TYPE shakeType = SHAKE_TYPE.RIGHT;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void start() {

        telemetry.clearAll();
        limelight.start();
    }

    @Override
    public void loop() {

        double shakeLeft = -SHAKE_DEGREES;
        double shakeRight = SHAKE_DEGREES;

        double shake;

        if (timer.milliseconds() >= SHAKE_FREQUENCY) {

            shakeType = shakeType.getBoolean() ? SHAKE_TYPE.LEFT : SHAKE_TYPE.RIGHT;
            timer.reset();
        }

        if (shakeType.getBoolean()) shake = shakeRight;
        else shake = shakeLeft;

        turret.setPosition((shake * ShooterInformation.ShooterConstants.TURRET_TICKS_PER_DEGREE) + startPosition);

        turret.update();

        if (tuningStage == TUNING_STAGE.POLL_RATE) {

            if (POLL_HZ_RATE != lastPollHzRate) limelight.setPollRateHz(POLL_HZ_RATE);
            lastPollHzRate = POLL_HZ_RATE;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) telemetry.addData("tx", result.getTx());
        }

        telemetry.addData("is result valid?", limelight.getLatestResult().isValid());
        telemetry.addData("result", limelight.getLatestResult().isValid());

        telemetry.addData("position", turret.getCurrentPosition());
        telemetry.addData("position error", turret.$getPositionError());
        telemetry.update();
    }

}
