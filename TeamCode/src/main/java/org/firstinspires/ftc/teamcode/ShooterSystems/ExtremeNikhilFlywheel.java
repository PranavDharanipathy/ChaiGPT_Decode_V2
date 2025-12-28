package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Objects;

public class ExtremeNikhilFlywheel {

    //Parameters in constructor
    DcMotorEx left_flywheel;
    DcMotorEx right_flywheel;

    Encoder encoder;




    public ExtremeNikhilFlywheel(DcMotorEx left_flywheel, DcMotorEx right_flywheel) {

        this.left_flywheel = left_flywheel;
        this.right_flywheel = right_flywheel;

        //encoder is only on left_flywheel

        encoder = new Encoder(left_flywheel);

        //set encoder direction to same direction as flywheel

        encoder.setDirection(Encoder.Direction.REVERSE);

        left_flywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[0]);
        right_flywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[1]);

        left_flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    double velocity;


    public void setVelocity(double velocity) {

        this.velocity = velocity;

        left_flywheel.setVelocity(velocity);

        right_flywheel.setVelocity(velocity);



    }





}
