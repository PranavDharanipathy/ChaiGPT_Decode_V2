package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.SimpleMathUtil;

public class TickrateChecker {

    private static double lastTime;
    private static double currentTime;

    public static void startOfLoop() {
        lastTime = System.nanoTime();
    }

    public static void endOfLoop() {
        currentTime = System.nanoTime();
    }

    public static double getTimePerTick() {
        return currentTime - lastTime;
    }

    /// Predicts CPU Usage at a low-level
    /// @return CPU Usage as a percentage
    public static double getTimeBasedCpuUsagePrediction() {
        return 100d * SimpleMathUtil.nanosecondsToMilliseconds((currentTime - lastTime)) / SimpleMathUtil.secondsToMilliseconds(Constants.CONTROL_HUB_HZ);
    }
}
