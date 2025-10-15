package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AdafruitBeambreakSensor {

    private DigitalChannel beambreak;

    public AdafruitBeambreakSensor(HardwareMap hardwareMap, String deviceName) {

        beambreak = hardwareMap.get(DigitalChannel.class, deviceName);

        beambreak.setMode(DigitalChannel.Mode.INPUT);
    }

    /// @return true when obstructed
    public boolean isBeamBroken() {
        // getState = true means not obstructed while false means that it is
        return !beambreak.getState();
    }
}