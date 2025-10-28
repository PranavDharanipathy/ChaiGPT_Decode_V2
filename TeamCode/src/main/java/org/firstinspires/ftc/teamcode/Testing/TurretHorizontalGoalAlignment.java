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
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;

@Config
@TeleOp (group = "testing")
public class TurretHorizontalGoalAlignment extends OpMode {

    public static int PIPELINE = 2;

    public static double ALIGNMENT_MULTIPLIER = 1;

    public static double TICKS_PER_DEGREE = 1; //it should include the turret gear ratio

    public static double CAMERA_TO_POINT_OF_ROTATION_2D = 1; //distance in inches

    private TurretBase turret;

    private Limelight3A limelight;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

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
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        double tx = result.getTx();
        double ty = result.getTy();

        double currentPosition = turret.getCurrentPosition();

        double flatDistance = ShooterInformation.Regressions.getDistanceFromRegression(ty);

        turret.setPosition(currentPosition + getPositionalIncrementToAlign(tx, flatDistance));
        turret.update();

        telemetry.addData("tx (y)", tx);
        telemetry.addData("regressed distance", "ty (z): %.4f, flat distance (x): %.4f", ty, flatDistance);
        telemetry.addData("current position", currentPosition);
        telemetry.update();
    }

    private double getPositionalIncrementToAlign(double tx, double flatDistanceFromCamera) {

        //purposefully ignores 3d distancing and uses 2d
        double halfCameraViewHorizontalDistanceInInches = Math.tan(tx / 2) * flatDistanceFromCamera;
        double totalDistance2d = CAMERA_TO_POINT_OF_ROTATION_2D + flatDistanceFromCamera;

        /* cameraViewHorizontalDistanceInInches
                      ___________
                      \    |    /
                       \   |   /
                        \  |--------totalDistance2d
                         \ | /
                          \|/
                      adjusted_tx

        cameraViewHorizontalDistanceInInches = 2(flatDistance * tan (0.5tx))

        adjusted_tx = 2 * tan^-1 (0.5cameraViewHorizontalDistanceInInches / totalDistance2d)

        */

        //isosceles triangle is split into a right-angle triangle
        double adjusted_tx = 2 * Math.atan(halfCameraViewHorizontalDistanceInInches / totalDistance2d);

        return ALIGNMENT_MULTIPLIER * adjusted_tx * TICKS_PER_DEGREE;
    }
}
