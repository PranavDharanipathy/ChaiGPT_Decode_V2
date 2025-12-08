package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public final class Intake extends Subsystem {

    private BetterGamepad controller1;
    private DcMotor intake;

    private AdafruitBeambreakSensor intakeBeambreak;

    private AdafruitBeambreakSensor transferBeambreak;

    public void provideComponents(DcMotor intake, AdafruitBeambreakSensor intakeBeambreak, AdafruitBeambreakSensor transferBeambreak, BetterGamepad controller1) {

        this.intake = intake;

        this.intakeBeambreak = intakeBeambreak;
        this.transferBeambreak = transferBeambreak;

        this.controller1 = controller1;
    }
    private boolean isBallInIntake = false;
    private boolean isBallInTransfer = false;

    private ElapsedTime isBallinIntakeDeadBandTimer = new ElapsedTime();

    private boolean isBallInIntakeDeadBandTriggered;

    private void beamBreakProcesses() {

        boolean RAW_isBallInIntake = intakeBeambreak.isBeamBroken().getBoolean();

        if (RAW_isBallInIntake) {
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

        isBallInIntake = intakeBeambreak.isBeamBroken().getBoolean();
        isBallInTransfer = transferBeambreak.isBeamBroken().getBoolean();
    }

    @Override
    public void update() {

        beamBreakProcesses();

        if (isBallInIntake) intake.setPower(Constants.INTAKE_POWER);

        //Intake and reverse-intake
        if (controller1.right_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setPower(Constants.INTAKE_POWER);
        } else if (controller1.left_trigger(Constants.TRIGGER_THRESHOLD)) {
            intake.setPower(Constants.REVERSE_INTAKE_POWER);
        } else if (!isBallInIntake) {
            intake.setPower(0);
        }

    }

}