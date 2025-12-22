package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class LiteralTransfer extends Subsystem {

    private BasicVeloMotor transfer;

    private BetterGamepad controller1;

    private AdafruitBeambreakSensor transferBeambreak;

    public void provideComponents(BasicVeloMotor transfer, AdafruitBeambreakSensor transferBeambreak, BetterGamepad controller1) {

        this.transfer = transfer;
        this.transferBeambreak = transferBeambreak;

        this.controller1 = controller1;
    }

    private boolean fullyTransfer = false;

    private final ElapsedTime fullyTransferTimer = new ElapsedTime();

    @Override
    public void update() {

        if (controller1.right_bumper()) {

            transfer.setVelocity(Constants.TRANSFER_VELOCITY);

            if (transferBeambreak.getBeamState().getBoolean()) {

                fullyTransfer = true;
                fullyTransferTimer.reset();
            }
        }
        else if (controller1.dpad_down()) {

            transfer.setVelocity(Constants.REVERSE_TRANSFER_VELOCITY);
        }
        else if (!fullyTransfer) {
            transfer.setVelocity(Constants.ANTI_TRANSFER_VELOCITY);
        }
        else if (fullyTransferTimer.milliseconds() > Constants.FULLY_TRANSFER_TIME) {
            fullyTransfer = false;
        }
        else {
            transfer.setVelocity(Constants.TRANSFER_VELOCITY);
        }
    }
}
