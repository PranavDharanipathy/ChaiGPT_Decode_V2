package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
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
    private boolean isBallInIntake = false;
    private boolean isBallInTransfer = false;

    private final ElapsedTime isBallinIntakeDeadBandTimer = new ElapsedTime();

    //private boolean isBallInIntakeDeadBandTriggered;
    private void beamBreakProcesses() {



        boolean RAW_isBallInIntake = intakeBeambreak.isBeamBroken().getBoolean();

        if (RAW_isBallInIntake) {
            isBallInIntake = true;
            //isBallInIntakeDeadBandTriggered = true;
            isBallinIntakeDeadBandTimer.reset();
        }

        else if (isBallinIntakeDeadBandTimer.milliseconds() < Constants.IS_BALL_IN_INTAKE_DEADBAND_TIMER) {
            isBallInIntake = true;
        }
        else {
            isBallInIntake = false;
            //isBallInIntakeDeadBandTriggered = false;


        }

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

        boolean reverseIntake = controller1.left_trigger(Constants.INTAKE_TRIGGER_THRESHOLD);

        //Intake and reverse-intake
        if (controller1.right_trigger(Constants.INTAKE_TRIGGER_THRESHOLD)) {
            intake.setVelocity(intakeVelocity);
        } else if (reverseIntake) {
            intake.setVelocity(Constants.REVERSE_INTAKE_VELOCITY);
        } else if (!isBallInIntake) {
            intake.setVelocity(0);
        }


        if (isBallInIntake && !reverseIntake) {
            intake.setVelocity(intakeVelocity);
        }

    }

}