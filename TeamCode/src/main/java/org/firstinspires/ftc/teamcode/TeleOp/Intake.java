package org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public final class Intake extends Subsystem {


    private BetterGamepad controller1;


    private BetterGamepad controller2;

    private BasicVeloMotor intake;
    private BasicVeloMotor transfer;

    // private DigitalChannel beam_break;

    public void provideComponents(BasicVeloMotor intake, BasicVeloMotor intakeBelt, BetterGamepad controller1, BetterGamepad controller2) {
        this.intake = intake;
    }


    @Override
    public void update() {
        intake.setVelocity(800);
        transfer.setVelocity(600);


        try {
            wait(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        intake.setVelocity(0);
        transfer.setVelocity(0);
    }
}