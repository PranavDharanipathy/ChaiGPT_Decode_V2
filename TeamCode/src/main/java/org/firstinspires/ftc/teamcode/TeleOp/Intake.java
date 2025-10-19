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

    public void provideComponents(BasicVeloMotor intake, AdafruitBeambreakSensor intakeBeambreak, AdafruitBeambreakSensor transferBeambreak, BetterGamepad controller1) {

        this.intake = intake;

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


        if (isBallInIntake) {
            intake.setVelocity(intakeVelocity);
        }





        //GENERAL TODOS FROM 10/17/2025 BELOW:


        //Nikhil TODO: fix positional PID Tuning

        //Nikhil TODO: I am not doing transfer, but I am still doing transfer beambreak.

        //Pranav TODO: When you click the shoot button, and it breaks the beambreak, it should move the ball to the shooter.
        //Pranav TODO: Only run transfer when people click the shoot button

        //Pranav: TODO: Get the shooter to align towards the apriltag when shooting.


        }

    }