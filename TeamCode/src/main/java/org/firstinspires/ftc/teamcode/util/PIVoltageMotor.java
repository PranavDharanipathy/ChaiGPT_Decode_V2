package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/// Mainly integrator run motor
/// <p>
/// Proportional term for higher responsiveness
public class PIVoltageMotor extends VMotor {

    public PIVoltageMotor(HardwareMap hardwareMap, String deviceName, Constants.HUB_TYPE hub) {

        super(hardwareMap, deviceName, hub);

        internalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setBrakeable(boolean brakeable) {

        if (brakeable) internalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else internalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setVelocityPICoefficients(double kp, double ki) {
        internalMotor.setVelocityPIDFCoefficients(kp, ki, 0, 0); // not using derivative or feedforward
    }

    public void setVelocity(double velocity) {
        internalMotor.setVelocity(velocity);
    }

    public double getPower() {
        return internalMotor.getPower();
    }

    public double getVelocity() { // ticks per second
        return internalMotor.getVelocity();
    }

}
