package org.firstinspires.ftc.teamcode.Tuners.DriveTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.TeleOpBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.PostAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

@Config
@TeleOp (group = "tuning")
public class FlywheelDriveTuning extends TeleOpBaseOpMode {

    public static double KP = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[0];
    public static double KI_FAR = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[1];
    public static double KI_CLOSE = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[2];
    public static double KD = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[3];
    public static double KV = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[4];
    public static double KS = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[5];
    public static double kPIDFUnitsPerVolt = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[6];
    public static double kISmash = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[7];
    public static double kISwitchError = Constants.FLYWHEEL_PIDVS_COEFFICIENTS[8];
    public static double I_MIN = Constants.FLYWHEEL_MIN_INTEGRAL_LIMIT, I_MAX = Constants.FLYWHEEL_MAX_INTEGRAL_LIMIT;
    public static double P_MIN = Constants.FLYWHEEL_MIN_PROPORTIONAL_LIMIT, P_MAX = Constants.FLYWHEEL_MAX_PROPORTIONAL_LIMIT;

    public static double TRANSFER_VELOCITY = 2000;
    public static double INTAKE_POWER = 1;
    public static double FLYWHEEL_VELOCITY = 405000;
    public static double HOOD_POSITION = 0.2;

    private Telemetry telemetry;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        initializeDevices();

        applyComponentTraits();

        //setup lynx module
        setUpLynxModule();

        if (isStopRequested()) return;
        waitForStart();

        //run robot reset
        RobotResetter robotReset = new PostAutonomousRobotReset(this);

        while (opModeIsActive()) {

            clearCacheOfLynxModule();

            flywheel.setIConstraints(I_MIN, I_MAX);
            flywheel.setPConstraints(P_MIN, P_MAX);
            flywheel.setVelocityPIDVSCoefficients(KP, KI_FAR, KI_CLOSE, KD, KV, KS, kPIDFUnitsPerVolt, kISmash, kISwitchError);

            hoodAngler.setPosition(HOOD_POSITION);
            intake.setPower(INTAKE_POWER);
            transfer.setVelocity(TRANSFER_VELOCITY);
            flywheel.setVelocity(FLYWHEEL_VELOCITY, true);
            flywheel.updateKvBasedOnVoltage();
            flywheel.update();

            telemetry.addData("Target Velocity", flywheel.getTargetVelocity());
            telemetry.addData("Real Velocity", flywheel.getRealVelocity());
            telemetry.addData("p", "%.5f", flywheel.p);
            telemetry.addData("i", "%.5f", flywheel.i);
            telemetry.addData("d", "%.5f", flywheel.d);
            telemetry.addData("v", "%.5f", flywheel.v);

            telemetry.addData("flywheel power", "%.5f", flywheel.getMotorPowers()[0]);
            telemetry.update();
        }
    }
}