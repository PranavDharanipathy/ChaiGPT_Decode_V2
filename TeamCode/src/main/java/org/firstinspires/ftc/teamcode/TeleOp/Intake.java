package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.ModifiedPIDFMotor;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public final class Intake extends Subsystem {

    private BetterGamepad controller1;
    private BasicVeloMotor intake;

    private AdafruitBeambreakSensor intakeBeambreak;

    private AdafruitBeambreakSensor transferBeambreak;
    BasicVeloMotor transferVelo;

    private ModifiedPIDFMotor transferPositional;

    public void provideComponents(BasicVeloMotor intake, BasicVeloMotor transferVelo, ModifiedPIDFMotor transferPositional, AdafruitBeambreakSensor intakeBeambreak, AdafruitBeambreakSensor transferBeambreak, BetterGamepad controller1) {

        this.intake = intake;
        this.transferVelo = transferVelo;
        this.transferPositional = transferPositional;

        this.intakeBeambreak = intakeBeambreak;
        this.transferBeambreak = transferBeambreak;

        this.controller1 = controller1;
    }
    private boolean isBallInIntake = false; //ball is well in the intake
    private boolean isBallInTransfer = false;

    private void beamBreakProcesses() {

        isBallInIntake = intakeBeambreak.isBeamBroken().getBoolean();
        isBallInTransfer = transferBeambreak.isBeamBroken().getBoolean();
    }

    private double intakeVelocity;

    private void intakePIDFAndVelocityProcesses() {

        //If one ball is in Intake and one ball is in transfer

        if (isBallInTransfer && isBallInIntake) {
            //does not integral
            intake.setVelocityPIDFCoefficients(
                    Constants.INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER[0],
                    Constants.INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER[1],
                    Constants.INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER[2],
                    Constants.INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER[3]
            );

            intakeVelocity = Constants.INTAKE_VELOCITY_WHEN_BALL_IN_TRANSFER;
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
        }
    }

    @Override
    public void update() {

        beamBreakProcesses();

        intakePIDFAndVelocityProcesses();

        //Start intake motor while going forward

        //Intake and reverse-intake
        if (controller1.right_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setVelocity(intakeVelocity);
        } else if (controller1.left_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setVelocity(Constants.REVERSE_INTAKE_VELOCITY);
        } else if (!isBallInIntake) {
            intake.setVelocity(0);
        }

//Need to tune a positional PID for transfer

        if (isBallInIntake && !isBallInTransfer) {
            transferVelo.setVelocity(Constants.TRANSFER_VELOCITY);
        }
        else {
            transferVelo.setVelocity(0);
        }

        if (isBallInIntake) {
            intake.setVelocity(intakeVelocity);
        }
        //
        }

    }