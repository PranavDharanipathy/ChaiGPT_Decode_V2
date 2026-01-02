package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.CommandUtils.Command;
import org.firstinspires.ftc.teamcode.util.CommandUtils.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.CommandUtils.InstantCommand;
import org.firstinspires.ftc.teamcode.util.CommandUtils.T;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Intake extends Subsystem {

    private BetterGamepad controller1;
    private BetterGamepad controller2;

    private IntakeMotor intake;
    private LiftPTO liftPTO;

    private AdafruitBeambreakSensor intakeBeambreak;

    private AdafruitBeambreakSensor transferBeambreak;

    public void provideComponents(IntakeMotor intake, LiftPTO liftPTO, AdafruitBeambreakSensor intakeBeambreak, AdafruitBeambreakSensor transferBeambreak, BetterGamepad controller1, BetterGamepad controller2) {

        this.intake = intake;

        this.liftPTO = liftPTO;

        this.intakeBeambreak = intakeBeambreak;
        this.transferBeambreak = transferBeambreak;

        this.controller1 = controller1;
        this.controller2 = controller2;
    }
    private boolean isBallInIntake = false;
    private boolean isBallInTransfer = false;

    private ElapsedTime isBallinIntakeDeadBandTimer = new ElapsedTime();

    private boolean isBallInIntakeDeadBandTriggered;

    private void beamBreakProcesses() {

        boolean rawIsBallInIntake = intakeBeambreak.getBeamState().getBoolean();

        if (rawIsBallInIntake) {
            isBallInIntake = true;
            isBallInIntakeDeadBandTriggered = true;
            isBallinIntakeDeadBandTimer.reset();
        }

        else if (isBallInIntakeDeadBandTriggered && isBallinIntakeDeadBandTimer.milliseconds() < Constants.IS_BALL_IN_INTAKE_DEADBAND_TIMER) {
            isBallInIntake = true;
        }
        else {
            isBallInIntake = false;
            isBallInIntakeDeadBandTriggered = false;
        }

        isBallInIntake = intakeBeambreak.getBeamState().getBoolean();
        isBallInTransfer = transferBeambreak.getBeamState().getBoolean();
    }

    enum MODE { INTAKE, LIFT }

    public MODE mode = MODE.INTAKE;

    @Override
    public void update() {

        beamBreakProcesses();

        if (controller2.optionsHasJustBeenPressed) {

            switch (mode) {

                case INTAKE: //switching to lift
                    mode = MODE.LIFT;
                    intake.setFunction(IntakeMotor.Function.LIFT);
                    liftPTO.setState(LiftPTO.PTOState.ENGAGE);
                    liftFirstInstance = true;
                    break;

                case LIFT: //switching to intake
                    mode = MODE.INTAKE;
                    intake.setFunction(IntakeMotor.Function.INTAKE);
                    liftPTO.setState(LiftPTO.PTOState.DISENGAGE);
                    break;
            }
        }

        if (isBallInIntake) intake.setPower(Constants.INTAKE_POWER);

        if (mode == MODE.LIFT) lift();
        else intake();

        intake.update();
    }

    private void intake() {

        //Intake and reverse-intake
        if (controller1.right_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setPower(Constants.INTAKE_POWER);
        } else if (controller1.left_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setPower(Constants.REVERSE_INTAKE_POWER);
        } else if (!isBallInIntake) {
            intake.setPower(0);
        }
    }

    private boolean liftFirstInstance;

    private void lift() {

        if (liftFirstInstance) {

            CommandScheduler.schedule(
                    new T(200),
                    new InstantCommand(() -> intake.setPosition(Constants.LIFT_POSITION))
            );

            liftFirstInstance = false;
        }
    }


    public boolean getLiftEngaged() {
        return liftPTO.getState() == LiftPTO.PTOState.ENGAGE;
    }

    public double getLiftPosition() {
        return intake.getReZeroedPosition();
    }

}