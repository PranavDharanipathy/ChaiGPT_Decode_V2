package org.firstinspires.ftc.teamcode.util;

/// A Low-Pass Filter used to smooth out noise.
public class LowPassFilter {

    public static double getFilteredValue(double previousFiltered, double raw, double alpha) {
        return (previousFiltered * (1 - alpha)) + alpha * raw;
    }
}
