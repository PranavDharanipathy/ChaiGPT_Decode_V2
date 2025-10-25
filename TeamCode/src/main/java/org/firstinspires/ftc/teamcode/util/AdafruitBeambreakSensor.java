package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AdafruitBeambreakSensor {

    private DigitalChannel receiver;
    private DigitalChannel power;

    public AdafruitBeambreakSensor(HardwareMap hardwareMap, String powerDevice, String receiverDevice) {

        receiver = hardwareMap.get(DigitalChannel.class, receiverDevice);
        power = hardwareMap.get(DigitalChannel.class, powerDevice);

        power.setMode(DigitalChannel.Mode.OUTPUT);

        receiver.setMode(DigitalChannel.Mode.INPUT);
    }

    /// 'true' when connected
    /// <p>
    /// 'false' when obstructed
    public enum BEAM_STATE {

        CONNECTED(true), BROKEN(false);

        private boolean beamState;

        BEAM_STATE(boolean beamState) {
            this.beamState = beamState;
        }

        public boolean getBoolean() {
            return beamState;
        }
    }

    public BEAM_STATE isBeamBroken() {

        BEAM_STATE state;

        // beambreak.getState() = true means connected while false means that it's obstructed
        if (!receiver.getState()) {
            //is connected
            state = BEAM_STATE.CONNECTED;
        }
        else {
            //is obstructed
            state = BEAM_STATE.BROKEN;
        }

        return state;
    }
}
