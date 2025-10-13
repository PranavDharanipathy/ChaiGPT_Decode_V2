package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants;

public class VMotor {

    /// exposed for child class usage
    DcMotorEx internalMotor;

    /// for object usage
    public VoltageSensor internalBatteryVoltageSensor;

    public VMotor(HardwareMap hardwareMap, String deviceName, Constants.HUB_TYPE hub) {

        internalMotor = hardwareMap.get(DcMotorEx.class, deviceName);

        internalBatteryVoltageSensor = hardwareMap.get(VoltageSensor.class, hub.getGivenName());
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        internalMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return internalMotor.getZeroPowerBehavior();
    }

    public void setMode(DcMotor.RunMode mode) {
        internalMotor.setMode(mode);
    }

    public DcMotor.RunMode getMode() {
        return internalMotor.getMode();
    }

    public void setDirection(DcMotor.Direction direction) {
        internalMotor.setDirection(direction);
    }

    public DcMotor.Direction getDirection() {
        return internalMotor.getDirection();
    }

    public void setVolts(double volts) {
        internalMotor.setPower(volts / internalBatteryVoltageSensor.getVoltage());
    }

    public void getVolts() {
        internalMotor.setPower(internalMotor.getPower() * internalBatteryVoltageSensor.getVoltage());
    }

}
