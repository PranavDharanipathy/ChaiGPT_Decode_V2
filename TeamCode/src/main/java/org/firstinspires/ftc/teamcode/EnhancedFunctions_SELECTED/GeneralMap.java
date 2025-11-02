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
    public <T extends HardwareDevice> T get(String deviceName) {
        return (T) hardwareMap.get(HardwareDevice.class, deviceName);
    }
}
