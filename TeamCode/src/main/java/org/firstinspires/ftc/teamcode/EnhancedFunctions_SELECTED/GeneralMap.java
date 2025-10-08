package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

@SuppressWarnings("unchecked")
public final class GeneralMap {

    private HardwareMap hardwareMap;

    public GeneralMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /// Errors will only be revealed at runtime through ClassCastException
    /// @apiNote This method is NOT type-safe
    public <Device extends HardwareDevice> Device get(String deviceName) {
        return (Device) hardwareMap.get(HardwareDevice.class, deviceName);
    }
}
