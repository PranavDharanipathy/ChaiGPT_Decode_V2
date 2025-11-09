package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.util.FastMath;

public strictfp class MathUtil {

    private MathUtil() {}

    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(value, max));
    }

    public static long clamp(long value, long min, long max) {
        return Math.max(min, Math.min(value, max));
    }

    public static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static Integer toInteger(int intValue) {
        return intValue;
    }

    public static Long toLong(long longValue) {
        return longValue;
    }

    public static Float toFloat(float floatValue) {
        return floatValue;
    }

    public static Double toDouble(double doubleValue) {
        return doubleValue;
    }

    public static double nanosecondsToMilliseconds(double nanoseconds) {
        return nanoseconds / 1_000_000.0;
    }

    public static double millisecondsToNanoseconds(double milliseconds) {
        return milliseconds * 1_000_000.0;
    }

    public static double secondsToNanoseconds(double seconds) {
        return seconds * 1_000_000_000.0;
    }

    public static double nanosecondsToSeconds(double nanoseconds) {
        return nanoseconds / 1_000_000_000.0;
    }

    public static double secondsToMilliseconds(double seconds) {
        return seconds * 1_000.0;
    }

    public static double millisecondsToSeconds(double milliseconds) {
        return milliseconds / 1_000.0;
    }

    public static double getDistance2d(double x1, double x2, double y1, double y2) {
        return FastMath.sqrt(FastMath.pow(x2 - x1, 2) + FastMath.pow(y2 - y1, 2));
    }

    public static double getDistance3d(double x1, double x2, double y1, double y2, double z1, double z2) {
        return FastMath.sqrt(FastMath.pow(x2 - x1, 2) + FastMath.pow(y2 - y1, 2) + FastMath.pow(z2 - z1, 2));
    }

    public static double metersToInches(double meters) {
        return meters * 39.3701;
    }

    public static double inchesToMillimeters(double inches) {
        return inches * 25.4;
    }
}
