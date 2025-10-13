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

    public static long nanosecondsToMilliseconds(long nanoseconds) {
        return nanoseconds / 1_000_000;
    }

    public static long millisecondsToNanoseconds(long milliseconds) {
        return milliseconds * 1_000_000;
    }

    public static long nanosecondsToSeconds(long nanoseconds) {
        return nanoseconds / 1_000_000_000;
    }

    public static double secondsToNanoseconds(double seconds) {
        return seconds * 1_000_000_000;
    }

    public static double nanosecondsToSeconds(double nanoseconds) {
        return nanoseconds / 1_000_000_000;
    }

    public static long secondsToMilliseconds(long seconds) {
        return seconds / 1_000;
    }

    public static long millisecondsToSeconds(long milliseconds) {
        return milliseconds * 1_000;
    }
}
