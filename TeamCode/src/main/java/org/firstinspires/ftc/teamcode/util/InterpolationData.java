package org.firstinspires.ftc.teamcode.util;


public class InterpolationData {

    /// First indexed value is the data pair, the second is x or y of the data pair.
    /// <p>
    /// |0| |1| is the y value of the first data pair.
    public double[][] dataPoints;

    /**
     * @param dataPoint1 minimum
     * @param dataPoint2 point in between minimum and maximum
     * @param dataPoint3 maximum
     **/
    public InterpolationData(double[] dataPoint1, double[] dataPoint2, double[] dataPoint3) {

        dataPoints = new double[][] {dataPoint1, dataPoint2, dataPoint3};
    }

    /**
     * @param dataPoint1 minimum
     * @param dataPoint2 maximum
     **/
    public InterpolationData(double[] dataPoint1, double[] dataPoint2) {

        dataPoints = new double[][] {dataPoint1, dataPoint2};
    }
}