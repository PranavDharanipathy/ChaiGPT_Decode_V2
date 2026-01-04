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

    public static long LOOP_TIME = 70;

    public static double KP = Constants.TURRET_PIDFS_COEFFICIENTS[0];
    public static double KI_OUT = Constants.TURRET_PIDFS_COEFFICIENTS[1];
    public static double KI_IN = Constants.TURRET_PIDFS_COEFFICIENTS[2];
    public static double KD = Constants.TURRET_PIDFS_COEFFICIENTS[3];
    public static double KF = 0;
    public static double KS = Constants.TURRET_PIDFS_COEFFICIENTS[5];

    public static double KI_SMASH = Constants.TURRET_PIDFS_COEFFICIENTS[6];

    public static double KI_SWITCH_ERROR = Constants.TURRET_PIDFS_COEFFICIENTS[7];

    public static double KD_FILTER = Constants.TURRET_PIDFS_COEFFICIENTS[8];
    public static double KPOWER_FILTER = Constants.TURRET_PIDFS_COEFFICIENTS[9];

    public static double LANYARD_EQUILIBRIUM = Constants.TURRET_PIDFS_COEFFICIENTS[10];

    public static double KF_DAMPEN = Constants.TURRET_PIDFS_COEFFICIENTS[11];

    public static double KVOLTAGE_FILTER = Constants.TURRET_PIDFS_COEFFICIENTS[12];

    public static double MIN_I = Constants.TURRET_MIN_INTEGRAL_LIMIT, MAX_I = Constants.TURRET_MAX_INTEGRAL_LIMIT;
    public static double TARGET_POSITION;

    public enum KF_MODE {
        MANUAL, INTERPOLATION
    }

    public static KF_MODE kfMode = KF_MODE.MANUAL;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretBase(hardwareMap);
        turret.setPIDFSCoefficients(KP, KI_OUT, KI_IN, KD, KF, KS, KI_SMASH, KI_SWITCH_ERROR, KD_FILTER, KPOWER_FILTER, LANYARD_EQUILIBRIUM, KF_DAMPEN, KVOLTAGE_FILTER);
    }

    @Override
    public void loop() {

        turret.setPosition(TARGET_POSITION);
        turret.setIConstraints(MIN_I, MAX_I);
        turret.setPIDFSCoefficients(KP, KI_OUT, KI_IN, KD, kfMode == KF_MODE.MANUAL ? KF : null, KS, KI_SMASH, KI_SWITCH_ERROR, KD_FILTER, KPOWER_FILTER, LANYARD_EQUILIBRIUM, KF_DAMPEN, KVOLTAGE_FILTER);
        turret.update();

        sleep(LOOP_TIME);

        telemetry.addData("kfScalingVoltage", turret.kfScalingVoltage);
        telemetry.addData("filteredVoltage", turret.filteredVoltage);
        telemetry.addData("real kf", turret.getRealKf());
        telemetry.addData("real ki", turret.getRealKi());
        telemetry.addData("f adjuster", turret.getFAdjuster());
        telemetry.addData("p", turret.p);
        telemetry.addData("i", turret.i);
        telemetry.addData("d", turret.d);
        telemetry.addData("f", turret.f);
        telemetry.addData("s", turret.s);
        telemetry.addData("position error", turret.getRawPositionError());
        telemetry.addData("current position", turret.getCurrentPosition());
        telemetry.addData("target position", turret.getTargetPosition());
        telemetry.addData("total power", turret.getServoPowers()[0]);
        telemetry.addData("start position", turret.startPosition);
        telemetry.update();

    }
}