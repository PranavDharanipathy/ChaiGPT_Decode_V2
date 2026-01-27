package org.firstinspires.ftc.teamcode.util.TelemetryUtils;

public enum TelemetryMode {

    DEBUG,
    INFO,
    RAW_DATA,
    AURA,
    MESSAGE,
    MISCELLANEOUS;

    public static TelemetryMode[] getAll() {

        return new TelemetryMode[] {
                DEBUG,
                INFO,
                RAW_DATA,
                AURA,
                MESSAGE,
                MISCELLANEOUS
        };
    }
}