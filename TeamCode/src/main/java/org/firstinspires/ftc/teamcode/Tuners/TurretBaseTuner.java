package org.firstinspires.ftc.teamcode.Tuners;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;

@Config
@TeleOp(group = "tuning")
public class TurretBaseTuner extends OpMode {

    private TurretBase turret;

    public static long LOOP_TIME = 20;

    public static double KP = Constants.TURRET_PIDF_COEFFICIENTS[0];
    public static double KI = Constants.TURRET_PIDF_COEFFICIENTS[1];
    public static double KD = Constants.TURRET_PIDF_COEFFICIENTS[2];
    public static double KF = Constants.TURRET_PIDF_COEFFICIENTS[3];

    public static double KI_SMASH = Constants.TURRET_PIDF_COEFFICIENTS[4];

    public static double MIN_I = Constants.TURRET_MIN_INTEGRAL_LIMIT, MAX_I = Constants.TURRET_MAX_INTEGRAL_LIMIT;
    public static double TARGET_POSITION;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretBase(hardwareMap);
        turret.setPIDFCoefficients(KP, KI, KD, KF, KI_SMASH);
    }

    @Override
    public void loop() {

        turret.setPosition(TARGET_POSITION);
        turret.setIConstraints(MIN_I, MAX_I);
        turret.updateCoefficients(KP, KI, KD, KF, KI_SMASH);
        turret.update();

        sleep(LOOP_TIME);

        telemetry.addData("p", turret.p);
        telemetry.addData("i", turret.i);
        telemetry.addData("d", turret.d);
        telemetry.addData("f", turret.ff);
        telemetry.addData("position error", turret.getPositionError());
        telemetry.addData("current position", turret.getCurrentPosition());
        telemetry.addData("target position", turret.getTargetPosition());
        telemetry.update();

    }
}