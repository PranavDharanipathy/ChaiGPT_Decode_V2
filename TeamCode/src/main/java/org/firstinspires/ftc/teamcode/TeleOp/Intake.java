package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
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

        boolean rawIsBallInIntake = !intakeBeambreak.getBeamState().getBoolean();

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

        //isBallInIntake = !intakeBeambreak.getBeamState().getBoolean();
        isBallInTransfer = !transferBeambreak.getBeamState().getBoolean();
    }

    public enum Stage {
        INTAKE, DRIVE_TO_BASE, LIFT
    }

    private Stage stage = Stage.INTAKE;

    public Stage getStage() {
        return stage;
    }

    @Override
    public void update() {

        beamBreakProcesses();

        if (controller2.right_stick_buttonHasJustBeenPressed) {
            toIntake();
        }
        else if (controller2.left_bumperHasJustBeenPressed) {

            switch (stage) {

                case INTAKE:
                    toDriveToBase();
                    break;

                case DRIVE_TO_BASE:
                    toLift();
                    break;

                case LIFT:
                    toIntake();
                    break;
            }
        }

        if (stage == Stage.LIFT) lift();
        else intake();

        intake.update();
    }

    private void intake() {

        //Intake and reverse-intake
        if (controller1.right_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setPower(Constants.INTAKE_POWER);
        }
        else if (controller1.left_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setPower(Constants.REVERSE_INTAKE_POWER);
        }
//        else if (isBallInIntake) {
//            intake.setPower(Constants.INTAKE_POWER);
//        }
        else {
            intake.setPower(0);
        }
    }

    private boolean liftFirstInstance;

    private void lift() {

        if (liftFirstInstance) {

            intake.setReZeroedPosition(Constants.LIFT_POSITION);
            liftFirstInstance = false;
        }
    }

    private void toIntake() {

        stage = Stage.INTAKE;
        intake.setFunction(IntakeMotor.Function.INTAKE);
        liftPTO.setState(LiftPTO.PTOState.DISENGAGE);
    }

    private void toDriveToBase() {

        stage = Stage.DRIVE_TO_BASE;
        intake.setFunction(IntakeMotor.Function.INTAKE);
        liftPTO.setState(LiftPTO.PTOState.DISENGAGE);
    }

    private void toLift() {

        stage = Stage.LIFT;
        intake.setFunction(IntakeMotor.Function.LIFT);
        liftPTO.setState(LiftPTO.PTOState.ENGAGE);
        liftFirstInstance = true;
    }

    public boolean getLiftEngaged() {
        return liftPTO.getState() == LiftPTO.PTOState.ENGAGE;
    }

    public double getLiftPosition() {
        return intake.getReZeroedPosition();
    }

}