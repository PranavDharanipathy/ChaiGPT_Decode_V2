package org.firstinspires.ftc.teamcode.Auto.autosubsystems;

import com.chaigptrobotics.shenanigans.No_u;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@No_u
public class TransferNF implements Subsystem {

    //Doesn't allow objects to be created
    private TransferNF() {}

    public static final TransferNF INSTANCE = new TransferNF();

    public BasicVeloMotor transfer;

    @Override
    public void initialize() {

        transfer = new BasicVeloMotor(ActiveOpMode.hardwareMap(), Constants.MapSetterConstants.transferMotorDeviceName);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Command transfer() {
        return new InstantCommand(() -> transfer.setVelocity(2300));
    }

    public Command antiStrong() {
        return new InstantCommand(() -> transfer.setVelocity(-450));
    }

    public Command antiVeryStrong() {
        return new InstantCommand(() -> transfer.setVelocity(-2000));
    }

    public Command antiNormal() {
        return new InstantCommand(() -> transfer.setVelocity(Constants.ANTI_TRANSFER_VELOCITY));
    }

    public Command idle() {
        return new InstantCommand(() -> transfer.setVelocity(0));
    }

    public void end() {
        transfer.setVelocity(0);
    }

    @Override
    public void periodic() {

    }
}
