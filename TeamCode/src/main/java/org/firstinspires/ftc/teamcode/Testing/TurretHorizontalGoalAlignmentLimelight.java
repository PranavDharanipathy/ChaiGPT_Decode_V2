package org.firstinspires.ftc.teamcode.Testing;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.chaigptrobotics.systems.DeprecatedSystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TickrateChecker;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Disabled
//@Config
@DeprecatedSystem(notes = "Turret used limelight to localize, however, we have switched to using odometry+imu with goBILDA Pinpoint")
@TeleOp (group = "testing")
public class TurretHorizontalGoalAlignmentLimelight extends OpMode {

    public enum ENCODER_TICK_DIRECTION {

        NORMAL(1), REVERSE(-1);

        private int direction;

        ENCODER_TICK_DIRECTION(int direction) {
            this.direction = direction;
        }

        public int getDirection() {
            return direction;
        }
    }

    public static ENCODER_TICK_DIRECTION encoderTickDirection = ENCODER_TICK_DIRECTION.REVERSE;

    public static long LOOP_TIME_DELAY = 0;

    public static int PIPELINE = 2;

    public static int POLL_HZ_RATE = ShooterInformation.CameraConstants.CAMERA_POLL_RATE;

    private double MIN_TURRET_POSITION, MAX_TURRET_POSITION;

    //normalized
    public static double MIN_TURRET_POSITION_IN_DEGREES = -170, MAX_TURRET_POSITION_IN_DEGREES = 170;

    public static double TICKS_PER_DEGREE = 73.5179487179; //it should include the turret gear ratio -> (encoder rotations per turret rotation) * (8192 / 360)

    public static double CAMERA_TO_POINT_OF_ROTATION_2D = ShooterInformation.CameraConstants.CAMERA_TO_POINT_OF_ROTATION_2D; //distance in inches

    private TurretBase turret;

    private Limelight3A limelight;

    private Telemetry telemetry;

    private BetterGamepad gamepad1;

    public enum TUNING_STAGE {
        RUNNING, PAUSED
    }

    public enum TX_TYPE {
        RAW, ADJUSTED
    }

    public static TUNING_STAGE tuningStage = TUNING_STAGE.RUNNING;

    public static TX_TYPE txType = TX_TYPE.RAW;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepad1 = new BetterGamepad(super.gamepad1);

        limelight = hardwareMap.get(Limelight3A.class, Constants.MapSetterConstants.limelight3AUSBDeviceName);

        limelight.setPollRateHz(POLL_HZ_RATE);
        limelight.pipelineSwitch(PIPELINE);

         turret = new TurretBase(hardwareMap);

         turret.setIConstraints(Constants.TURRET_MIN_INTEGRAL_LIMIT, Constants.TURRET_MAX_INTEGRAL_LIMIT);
         turret.setPIDFCoefficients(
                 Constants.TURRET_PIDF_COEFFICIENTS[0],
                 Constants.TURRET_PIDF_COEFFICIENTS[1],
                 Constants.TURRET_PIDF_COEFFICIENTS[2],
                 Constants.TURRET_PIDF_COEFFICIENTS[3],
                 Constants.TURRET_PIDF_COEFFICIENTS[4],
                 Constants.TURRET_PIDF_COEFFICIENTS[5],
                 Constants.TURRET_PIDF_COEFFICIENTS[6]
         );

    }



    @Override
    public void start() {

        startPosition = turret.getCurrentPosition();
        position = startPosition;

        limelight.start();
    }

    private double startPosition;

    private double getRezeroedPosition() {
        return turret.getCurrentPosition() - startPosition;
    }

    private double lastTx;
    private double tx;
    private double position;

    @Override
    public void loop() {

        int directionalMultiplier = encoderTickDirection.getDirection();

        MIN_TURRET_POSITION = MIN_TURRET_POSITION_IN_DEGREES * TICKS_PER_DEGREE;
        MAX_TURRET_POSITION = MAX_TURRET_POSITION_IN_DEGREES * TICKS_PER_DEGREE;



        gamepad1.getInformation();

        LLResult result = limelight.getLatestResult();

        Double ty;

        Double flatDistance;

        if (result != null && result.isValid()) {

            ty = result.getTy();
            flatDistance = ShooterInformation.Regressions.getDistanceFromRegression(ty);
        }
        else {

            ty = null;
            flatDistance = null;
        }

        if (result != null && result.isValid()) {

            lastTx = tx;

            if (txType == TX_TYPE.RAW) {
                tx = directionalMultiplier * result.getTx();
            } else {
                tx = directionalMultiplier * getAdjustedTx(result.getTx(), flatDistance);
            }
        }

        double currentPosition = turret.getCurrentPosition();

        if (result != null && result.isValid()) {
            position = currentPosition + (tx * TICKS_PER_DEGREE);
        }
        else {
            position = currentPosition + (lastTx * TICKS_PER_DEGREE);
        }

        turret.setPosition(MathUtil.clamp(position, MIN_TURRET_POSITION + startPosition, MAX_TURRET_POSITION + startPosition));

        if (tuningStage == TUNING_STAGE.RUNNING) turret.update();

        telemetry.addData("position", position);
        telemetry.addData("start position", startPosition);
        telemetry.addData("rezeroed position", getRezeroedPosition());
        telemetry.addData("min position", MIN_TURRET_POSITION);
        telemetry.addData("max position", MAX_TURRET_POSITION);

        telemetry.addData("tx", "raw: %.4f, adjusted: %.4f", result != null && result.isValid() ? result.getTx() : null, result != null && result.isValid() ? getAdjustedTx(result.getTx(), flatDistance) : null);
        telemetry.addData("regressed distance", "ty (z): %.4f, flat distance (x): %.4f", ty, flatDistance);
        telemetry.addData("current position", currentPosition);
        telemetry.addData("position error", turret.getPositionError());

        telemetry.addData("power", turret.$getServoPowers()[0]);

        telemetry.addData("Tick rate", TickrateChecker.getTimePerTick());
        telemetry.update();

        sleep(LOOP_TIME_DELAY);
    }

    private double getAdjustedTx(double tx, Double flatDistanceFromCamera) {

        double x = Math.tan(Math.toRadians(tx)) * flatDistanceFromCamera;

        double adjusted_tx = Math.atan(x / (flatDistanceFromCamera + CAMERA_TO_POINT_OF_ROTATION_2D));

        telemetry.addData("adjusted_tx", adjusted_tx);

        return adjusted_tx;
    }
}
