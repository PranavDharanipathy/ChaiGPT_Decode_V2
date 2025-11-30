package org.firstinspires.ftc.teamcode.util;

public class LinearInterpolator {

    private LinearInterpolator() {}

    public static double estimateValue(double x, InterpolationData data) {

        if (data.dataPoints.length != 2) throw new IllegalArgumentException("Interpolation data must have 2 data points.");

        double x0 = data.dataPoints[0][0];
        double x1 = data.dataPoints[1][0];

        double y0 = data.dataPoints[0][1];
        double y1 = data.dataPoints[1][1];

        return y0 + (y1 - y0) * ((x - x0) / (x1 - x0));
    }
}