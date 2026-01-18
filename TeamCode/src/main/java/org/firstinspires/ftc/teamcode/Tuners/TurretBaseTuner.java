package org.firstinspires.ftc.teamcode.Tuners;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.Constants.TURRET_PIDFS_COEFFICIENTS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBasePIDFSCoefficients;

@Config
@TeleOp(group = "tuning")
public class TurretBaseTuner extends OpMode {

    private TurretBase turret;

    public static long LOOP_TIME = 70;

    public static double KP =TURRET_PIDFS_COEFFICIENTS.kp;
    public static double[] KI_FAR = {TURRET_PIDFS_COEFFICIENTS.lkiFar, TURRET_PIDFS_COEFFICIENTS.rkiFar};
    public static double[] KI_CLOSE = {TURRET_PIDFS_COEFFICIENTS.lkiClose, TURRET_PIDFS_COEFFICIENTS.rkiClose};
    public static double KD = TURRET_PIDFS_COEFFICIENTS.kd;
    public static double KF = 0;
    public static double KS = TURRET_PIDFS_COEFFICIENTS.ks;

    public static double[] I_SWITCH = {TURRET_PIDFS_COEFFICIENTS.lISwitch, TURRET_PIDFS_COEFFICIENTS.rISwitch};

    public static double[] KI_SMASH = {TURRET_PIDFS_COEFFICIENTS.lkISmash, TURRET_PIDFS_COEFFICIENTS.rkISmash};

    public static double[] KD_FILTER = {TURRET_PIDFS_COEFFICIENTS.lkDFilter, TURRET_PIDFS_COEFFICIENTS.rkDFilter};
    public static double KPOWER_FILTER = TURRET_PIDFS_COEFFICIENTS.kPowerFilter;

    public static double[] D_ACTIVATION = {TURRET_PIDFS_COEFFICIENTS.lDActivation, TURRET_PIDFS_COEFFICIENTS.rDActivation};

    public static double KF_RESISTANCE = TURRET_PIDFS_COEFFICIENTS.kFResistance;

    public static double MIN_I = TURRET_PIDFS_COEFFICIENTS.minI, MAX_I = TURRET_PIDFS_COEFFICIENTS.maxI;
    public static double TARGET_POSITION;

    public static String PD_MODE = "00";

    public enum FMODE {
        MANUAL, INTERPOLATION
    }

    public static FMODE kfMode = FMODE.MANUAL;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretBase(hardwareMap);
        turret.setPIDFSCoefficients(new TurretBasePIDFSCoefficients(
                KP,
                KI_FAR,
                KI_CLOSE,
                KD,
                KF,
                KS,
                I_SWITCH,
                KI_SMASH,
                D_ACTIVATION,
                KD_FILTER,
                KPOWER_FILTER,
                KF_RESISTANCE,
                MIN_I, MAX_I
        ));
        turret.setTuning(true);
    }

    @Override
    public void loop() {

        turret.setPdInterpolationMode(TurretBase.PD_INTERPOLATION_MODE.fromString(PD_MODE));

        turret.setPosition(TARGET_POSITION);
        turret.setPIDFSCoefficients(new TurretBasePIDFSCoefficients(
                KP,
                KI_FAR,
                KI_CLOSE,
                KD,
                kfMode == FMODE.INTERPOLATION ? null : KF,
                KS,
                I_SWITCH,
                KI_SMASH,
                D_ACTIVATION,
                KD_FILTER,
                KPOWER_FILTER,
                KF_RESISTANCE,
                MIN_I, MAX_I
        ));
        turret.update();

        sleep(LOOP_TIME);

        telemetry.addData("ki", "%.8f", turret.ki);
        telemetry.addData("kd", "%.8f", turret.kd);
        telemetry.addData("kf", "%.8f", turret.kf);
        telemetry.addData("p", "%.5f", turret.p);
        telemetry.addData("i", "%.5f", turret.i);
        telemetry.addData("d", "%.5f", turret.d);
        telemetry.addData("f", "%.5f", turret.f);
        telemetry.addData("s", "%.5f", turret.s);
        telemetry.addData("position error", turret.getRawPositionError());
        telemetry.addData("current position", turret.getCurrentPosition());
        telemetry.addData("target position", turret.getTargetPosition());
        telemetry.addData("total power", turret.getServoPowers()[0]);
        telemetry.addData("start position", turret.startPosition);
        telemetry.update();

    }
}