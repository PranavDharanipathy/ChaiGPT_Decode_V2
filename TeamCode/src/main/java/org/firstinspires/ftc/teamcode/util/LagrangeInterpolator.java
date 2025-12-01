package org.firstinspires.ftc.teamcode.util;

public class LagrangeInterpolator {

    private LagrangeInterpolator() {}

    public static double estimateValue(double x, InterpolationData data) {

        if (data.dataPoints.length != 3) throw new IllegalArgumentException("Interpolation data must have 3 data points.");

        double x1 = data.dataPoints[0][0];
        double x2 = data.dataPoints[1][0];
        double x3 = data.dataPoints[2][0];

        double y1 = data.dataPoints[0][1];
        double y2 = data.dataPoints[1][1];
        double y3 = data.dataPoints[2][1];

        double L1 =
                ((x - x2)*(x - x3))
                /
                ((x1 - x2)*(x1 - x3));

        double L2 =
                ((x - x1)*(x - x3))
                /
                ((x2 - x1)*(x2 - x3));

        double L3 =
                ((x - x1)*(x - x2))
                /
                ((x3 - x1)*(x3 - x2));

        return (y1 * L1) + (y2 * L2) + (y3 * L3);
    }
}