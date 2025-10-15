package org.firstinspires.ftc.teamcode.util;

public class SimpleMathUtil {

    private SimpleMathUtil() {}

    public static int clamp(int value, int min, int max) {
        return Math.max(value, Math.min(value, max));
    }

    public static long clamp(long value, long min, long max) {
        return Math.max(value, Math.min(value, max));
    }

    public static float clamp(float value, float min, float max) {
        return Math.max(value, Math.min(value, max));
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(value, Math.min(value, max));
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
}
