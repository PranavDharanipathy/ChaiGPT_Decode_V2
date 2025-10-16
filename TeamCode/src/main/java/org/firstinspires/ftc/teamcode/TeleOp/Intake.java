package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public final class Intake extends Subsystem {

    private BetterGamepad controller1;
    private BasicVeloMotor intake;
    private BasicVeloMotor transfer;

    public void provideComponents(BasicVeloMotor intake, BetterGamepad controller1, BetterGamepad controller2) {

        this.intake = intake;
        this.transfer = transfer;
        this.controller1 = controller1;

    }
    private boolean isBallToBeTransferred = false; //ball has not yet been transferred
    private boolean isBallInIntake = false; //ball is well in the intake
    private boolean isFullManualIntakeAllowed = true;
    private boolean isBallReadyToBeShot = false;

    private void beamBreakProcesses() {

        if (isBallReadyToBeShot) {

            isFullManualIntakeAllowed = true;
            isBallInIntake = false;
            isBallToBeTransferred = false;
        }
    }

    @Override
    public void update() {

        beamBreakProcesses();

        //Start intake motor while going forward

        //Intake
        if (controller1.right_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setVelocity(Constants.INTAKE_VELOCITY);

            //Outtake code
        } else if (controller1.left_trigger(Constants.TRIGGER_THRESHOLD) && isFullManualIntakeAllowed) {
            intake.setVelocity(Constants.REVERSE_INTAKE_VELOCITY);
        } else if (isFullManualIntakeAllowed) {
            intake.setVelocity(0);
        }

        if (isBallInIntake) {

            isFullManualIntakeAllowed = false;
            intake.setVelocity(Constants.INTAKE_VELOCITY);
            if (isBallToBeTransferred) transfer.setVelocity(Constants.TRANSFER_VELOCITY);

        }

        //Transfer --> Shooter

        else if (isBallReadyToBeShot) {
            transfer.setVelocity(Constants.TRANSFER_VELOCITY);
            intake.setVelocity(Constants.INTAKE_VELOCITY);


        }

        //Pranav's domain






    }
}