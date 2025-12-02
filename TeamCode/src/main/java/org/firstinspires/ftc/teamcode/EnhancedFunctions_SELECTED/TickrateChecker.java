package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class TickrateChecker {

    private static double lastTime;
    private static double currentTime;

    /// Also does the calculation
    public static double getTimePerTick() {

        lastTime = currentTime;
        currentTime = System.currentTimeMillis();
        return currentTime - lastTime;
    }

    /// Predicts CPU Usage at a low-level
    /// @return CPU Usage as a percentage
    public static double getRunSpeedPercentage() {
        return (currentTime - lastTime) / (Constants.CONTROL_HUB_HZ / 1000);
    }
}
