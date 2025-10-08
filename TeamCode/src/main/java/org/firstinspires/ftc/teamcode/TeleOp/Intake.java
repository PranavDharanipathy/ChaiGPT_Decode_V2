package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIVoltageMotor;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class Intake extends Subsystem {
//Initialized sub-systems
    private PIVoltageMotor intake;

    private BetterGamepad gamepad1;



    public void provideComponents(PIVoltageMotor intake, BetterGamepad gamepad1) {
        this.intake = intake;
        this.gamepad1 = gamepad1;
    }

    @Override
    public void update() {





    }
}
