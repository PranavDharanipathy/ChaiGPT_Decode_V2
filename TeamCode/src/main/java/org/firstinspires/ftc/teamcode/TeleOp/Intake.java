package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.TRANSFER_SLOW_REVERSE_VELOCITY;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public final class Intake extends Subsystem {

    private BetterGamepad controller1;
    private BasicVeloMotor intake;
    private BasicVeloMotor transfer;

    private AdafruitBeambreakSensor intakeBeambreak;
    private AdafruitBeambreakSensor transferBeambreak;


    public void provideComponents(BasicVeloMotor intake, BasicVeloMotor transfer, AdafruitBeambreakSensor intakeBeambreak, AdafruitBeambreakSensor transferBeambreak, BetterGamepad controller1) {

        this.controller1 = controller1;
        this.intake = intake;
        this.transfer = transfer;

        this.intakeBeambreak = intakeBeambreak;
        this.transferBeambreak = transferBeambreak;
    }
    private boolean isBallInIntake = false; //ball is well in the intake
    private boolean isBallInTransfer = false;

    private void beamBreakProcesses() {

        isBallInIntake = intakeBeambreak.isBeamBroken().getBoolean();
        isBallInTransfer = transferBeambreak.isBeamBroken().getBoolean();
    }

    private double intakeVelocity;
    private double transferVelocity;

    private void intakePIDFAndVelocityProcesses() {

        if (isBallInTransfer && isBallInIntake) {
            //does not integral
            intake.setVelocityPIDFCoefficients(
                    Constants.INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER[0],
                    Constants.INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER[1],
                    Constants.INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER[2],
                    Constants.INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER[3]
            );

            intakeVelocity = Constants.INTAKE_VELOCITY_WHEN_BALL_IN_TRANSFER;
            transferVelocity = 0; // stop transfer, already full
        }
        else {
            //uses integral
            intake.setVelocityPIDFCoefficients(
                    Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[0],
                    Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[1],
                    Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[2],
                    Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[3]
            );

            intakeVelocity = Constants.BASE_INTAKE_VELOCITY;
            transferVelocity = Constants.TRANSFER_VELOCITY;
        }
    }

    @Override
    public void update() {

        beamBreakProcesses();
        intakePIDFAndVelocityProcesses();

        //Start intake motor while going forward

        //Intake and reverse-intake
//        if (controller1.right_trigger(Constants.TRIGGER_THRESHOLD)) {
//            intake.setVelocity(intakeVelocity);
//        } else if (controller1.left_trigger(Constants.TRIGGER_THRESHOLD)) {
//            intake.setVelocity(Constants.REVERSE_INTAKE_VELOCITY);
//        } else {
//            intake.setVelocity(0);
//        }

        beamBreakProcesses();
        intakePIDFAndVelocityProcesses();

        // ---- Intake / Transfer logic ----
        if (controller1.right_trigger(Constants.TRIGGER_THRESHOLD)) {

            // --- Transfer motor behavior ---
            if (isBallInTransfer) {
                // Transfer sensor broken — run slowly backwards to prevent new ball from pushing in
                transfer.setVelocity(Constants.TRANSFER_SLOW_REVERSE_VELOCITY);
            } else {
                // Normal transfer velocity if transfer not full
                transfer.setVelocity(Constants.TRANSFER_VELOCITY);
            }

            // --- Intake motor behavior ---
            if (isBallInIntake) {
                // Intake sensor broken — ball is partially in
                // Run intake slowly to hold balls in place
                intake.setVelocity(Constants.BASE_INTAKE_VELOCITY * 0.10); // ~30% of full speed
            } else {
                // Normal intake velocity if intake empty
                intake.setVelocity(Constants.BASE_INTAKE_VELOCITY);
            }

        } else if (controller1.left_trigger(Constants.TRIGGER_THRESHOLD)) {
            // Reverse both to eject balls
            intake.setVelocity(Constants.REVERSE_INTAKE_VELOCITY);
            transfer.setVelocity(Constants.REVERSE_TRANSFER_VELOCITY);

        } else {
            // No triggers pressed — stop both
            intake.setVelocity(0);
            transfer.setVelocity(0);
        }
    }


        // --| If one ball is in transfer, and one ball is in intake |-- \\

}

