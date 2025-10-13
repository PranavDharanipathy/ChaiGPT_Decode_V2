package org.firstinspires.ftc.teamcode.TeleOp;




import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;


public class Intake extends Subsystem {


    private BetterGamepad controller;


    private BasicVeloMotor intake;
    private BasicVeloMotor transfer;


    // private DigitalChannel beam_break;


    public void provideComponents(BasicVeloMotor intake, BasicVeloMotor intakeBelt, BetterGamepad controller) {
        this.controller = controller;
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

