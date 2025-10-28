package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Config
@TeleOp (group = "testing")
public class TurretHorizontalGoalAlignment extends OpMode {

    public static int PIPELINE = 2;

    public static double ALIGNMENT_MULTIPLIER = 1;
    public static double OVERSHOOT_MULTIPLIER = 10;

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
        AUTOAIM, TURRET_LIMITS
    }

    public static TUNING_STAGE tuningStage = TUNING_STAGE.AUTOAIM;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepad1 = new BetterGamepad(super.gamepad1);

        limelight = hardwareMap.get(Limelight3A.class, Constants.MapSetterConstants.limelight3AUSBDeviceName);

        limelight.setPollRateHz(ShooterInformation.CameraConstants.CAMERA_POLL_RATE);
        limelight.pipelineSwitch(PIPELINE);

         turret = new TurretBase(hardwareMap, Constants.MapSetterConstants.turretBaseLeftServoDeviceName, Constants.MapSetterConstants.turretBaseRightServoDeviceName);

         turret.setIConstraints(Constants.TURRET_MIN_INTEGRAL_LIMIT, Constants.TURRET_MAX_INTEGRAL_LIMIT);
         turret.setPIDFCoefficients(
                 Constants.TURRET_PIDF_COEFFICIENTS[0],
                 Constants.TURRET_PIDF_COEFFICIENTS[1],
                 Constants.TURRET_PIDF_COEFFICIENTS[2],
                 Constants.TURRET_PIDF_COEFFICIENTS[3]
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
    private volatile double position;

    private double positionalSwitchValue = 0;

    private boolean positionalSwitch = false;

    @Override
    public void loop() {

        MIN_TURRET_POSITION = MIN_TURRET_POSITION_IN_DEGREES * TICKS_PER_DEGREE;
        MAX_TURRET_POSITION = MAX_TURRET_POSITION_IN_DEGREES * TICKS_PER_DEGREE;




        gamepad1.getInformation();

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) lastTx = tx;
        tx = result.getTx();
        double ty = result.getTy();

        double currentPosition = turret.getCurrentPosition();

        double flatDistance = ShooterInformation.Regressions.getDistanceFromRegression(ty);

        //double positionalIncrement = getPositionalIncrementToAlign(tx, flatDistance);
        //double position = currentPosition + positionalIncrement;

        if (result.isValid()) {
            position = positionalSwitchValue + currentPosition + ALIGNMENT_MULTIPLIER * (tx * TICKS_PER_DEGREE);
        }
        else {
            position = positionalSwitchValue + currentPosition + OVERSHOOT_MULTIPLIER * (lastTx * TICKS_PER_DEGREE);
        }

        //turret.setPosition(MathUtil.clamp(position, MIN_TURRET_POSITION, MAX_TURRET_POSITION));
        turret.setPosition(MathUtil.clamp(position, MIN_TURRET_POSITION + startPosition, MAX_TURRET_POSITION + startPosition));

        //telemetry.addData("positionalIncrement", positionalIncrement);
        telemetry.addData("positional switch?", positionalSwitch);
        telemetry.addData("positional switch value", positionalSwitchValue);
        telemetry.addData("rezeroed position", getRezeroedPosition());
        telemetry.addData("position", position);
        telemetry.addData("start position", startPosition);
        telemetry.addData("min position", MIN_TURRET_POSITION);
        telemetry.addData("max position", MAX_TURRET_POSITION);

//        if (gamepad1.aHasJustBeenPressed) turret.setPosition(position);
//        else if (gamepad1.bHasJustBeenPressed) turret.setPosition(currentPosition + (tx * TICKS_PER_DEGREE));
        if (tuningStage == TUNING_STAGE.AUTOAIM) turret.update();

        telemetry.addData("tx (y)", tx);
        telemetry.addData("regressed distance", "ty (z): %.4f, flat distance (x): %.4f", ty, flatDistance);
        telemetry.addData("current position", currentPosition);
        telemetry.update();
    }

//    private double getPositionalIncrementToAlign(double tx, double flatDistanceFromCamera) {
//
//        //purposefully ignores 3d distancing and uses 2d
//        double halfCameraViewHorizontalDistanceInInches = Math.tan(tx / 2) * flatDistanceFromCamera;
//        double totalDistance2d = CAMERA_TO_POINT_OF_ROTATION_2D + flatDistanceFromCamera;
//
//        /* cameraViewHorizontalDistanceInInches
//                      ___________
//                      \    |    /
//                       \   |   /
//                        \  |--------totalDistance2d
//                         \ | /
//                          \|/
//                      adjusted_tx
//
//        cameraViewHorizontalDistanceInInches = 2(flatDistance * tan (0.5tx))
//
//        adjusted_tx = 2 * tan^-1 (0.5cameraViewHorizontalDistanceInInches / totalDistance2d)
//
//        */
//
//        //isosceles triangle is split into a right-angle triangle
//        double adjusted_tx = 2 * Math.atan(halfCameraViewHorizontalDistanceInInches / totalDistance2d);
//
//        telemetry.addData("adjusted_tx", adjusted_tx);
//
//        return ALIGNMENT_MULTIPLIER * adjusted_tx * TICKS_PER_DEGREE;
//    }
}
