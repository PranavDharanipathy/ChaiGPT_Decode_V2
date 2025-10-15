package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public final class Intake extends Subsystem {


    private BetterGamepad controller1;


    private BetterGamepad controller2;

    private BasicVeloMotor intake;
    private BasicVeloMotor transfer;

    private BasicVeloMotor left_front;

    private BasicVeloMotor left_back;

    private BasicVeloMotor right_front;
    private BasicVeloMotor right_back;




    // private DigitalChannel beam_break;

    public void provideComponents(BasicVeloMotor intake, BasicVeloMotor intakeBelt, BetterGamepad controller1, BetterGamepad controller2) {
        this.intake = intake;
    }


    @Override
    public void update() {
        //Start intake motor while going forward
        intake.setVelocity(1200);
        //move forward

        left_front.setVelocity(200);
        left_back.setVelocity(200);
        right_front.setVelocity(200);
        right_back.setVelocity(200);

        WaitCommand.Wait(1500);
        left_front.setVelocity(-200);
        left_back.setVelocity(-200);
        right_front.setVelocity(-200);
        right_back.setVelocity(-200);

        WaitCommand.Wait(1500);









        try {
            wait(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        intake.setVelocity(0);
        transfer.setVelocity(0);
    }
}