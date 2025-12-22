package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.IntakeMotor;
import org.firstinspires.ftc.teamcode.TeleOp.LiftPTO;

@Config
@TeleOp(group = "tuning")
public class LiftTuner extends OpMode {

    private LiftPTO liftPTO;

    private IntakeMotor motor;

    public static double POSITION = 500;

    public static double POWER_DEBUG = 0;

    public static double KP = Constants.LIFT_PIDFS_COEFFICIENTS[0];
    public static double KI = Constants.LIFT_PIDFS_COEFFICIENTS[1];
    public static double KD = Constants.LIFT_PIDFS_COEFFICIENTS[2];
    public static double KF = Constants.LIFT_PIDFS_COEFFICIENTS[3];
    public static double KS = Constants.LIFT_PIDFS_COEFFICIENTS[4];
    public static double KI_SMASH = Constants.LIFT_PIDFS_COEFFICIENTS[5];
    public static double KD_FILTER = Constants.LIFT_PIDFS_COEFFICIENTS[6];
    public static double KOUTPUT_FILTER = Constants.LIFT_PIDFS_COEFFICIENTS[7];
    public static double LIFT_TOUCHES_GROUND_POSITION = Constants.LIFT_PIDFS_COEFFICIENTS[8];
    public static IntakeMotor.Function FUNCTION = IntakeMotor.Function.LIFT;

    public static double MIN_I = Constants.LIFT_MIN_INTEGRAL_LIMIT, MAX_I = Constants.LIFT_MAX_INTEGRAL_LIMIT;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        liftPTO = new LiftPTO(hardwareMap);

        motor = new IntakeMotor(hardwareMap);
    }

    @Override
    public void loop() {

        motor.setFunction(FUNCTION);

        if (FUNCTION == IntakeMotor.Function.LIFT) {
            liftPTO.setState(LiftPTO.PTOState.ENGAGE);
        }
        else {
            liftPTO.setState(LiftPTO.PTOState.DISENGAGE);
        }

        motor.setLiftPIDFSCoefficients(KP, KI, KD, KF, KS, KI_SMASH, KD_FILTER, KOUTPUT_FILTER, LIFT_TOUCHES_GROUND_POSITION);
        motor.setLiftIConstraints(MIN_I, MAX_I);

        motor.setPosition(POSITION);

        motor.setPower(POWER_DEBUG);

        motor.update();

        telemetry.addData("target position", motor.getTargetPosition());
        telemetry.addData("current position", motor.getReZeroedPosition());
        telemetry.addData("position error", motor.getPositionalError());
        telemetry.addData("volts", motor.getVolts());
        telemetry.addData("p", motor.p);
        telemetry.addData("i", motor.i);
        telemetry.addData("d", motor.d);
        telemetry.addData("f", motor.f);
        telemetry.addData("s", motor.s);
        telemetry.addData("power", motor.getPower());

        telemetry.addData("lift pto state", liftPTO.getState());
        telemetry.addData("intake motor function", motor.getFunction());
        telemetry.update();
    }
}
