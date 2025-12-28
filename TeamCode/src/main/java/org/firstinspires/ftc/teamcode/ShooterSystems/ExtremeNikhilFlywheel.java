package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
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

        left_flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right_flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        left_flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




    }

    double velocity;


    public void setVelocity(double velocity) {

        this.velocity = velocity;

        left_flywheel.setVelocity(velocity);

        right_flywheel.setVelocity(velocity);
    }

    public void stop() {

        left_flywheel.setPower(0);
        right_flywheel.setPower(0);
    }









}
