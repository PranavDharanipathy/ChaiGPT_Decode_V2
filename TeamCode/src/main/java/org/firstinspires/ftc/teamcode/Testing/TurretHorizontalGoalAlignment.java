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

    public static double CAMERA_TO_POINT_OF_ROTATION = 1; //distance in inches

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

        double flatDistanceFromGoal = ShooterInformation.Regressions.getDistanceFromRegression(ty);

        double currentPosition = turret.getCurrentPosition();

        turret.setPosition(currentPosition + getPositionalIncrementToAlign(tx));
        turret.update();

        telemetry.addData("tx", tx);
        telemetry.addData("regressed distance", "ty (x): %.4f, flat distance (y): %.4f", ty, flatDistanceFromGoal);
        telemetry.addData("current position", currentPosition);
        telemetry.update();
    }

    private double getPositionalIncrementToAlign(double tx) {

        /*             tx
                   __________
             theta \        / theta
                    \      /
                     \    /----CAMERA_TO_POINT_OF_ROTATION
                      \  /
                       \/
                  adjusted_tx

        cos theta = 0.5tx / CAMERA_TO_POINT_OF_ROTATION

        adjusted_tx = 2 * cos^-1 (0.5tx / CAMERA_TO_POINT_OF_ROTATION)

        * */

        double adjusted_tx = 2 * Math.acos((tx / 2) / CAMERA_TO_POINT_OF_ROTATION);

        return ALIGNMENT_MULTIPLIER * adjusted_tx * TICKS_PER_DEGREE;
    }
}
