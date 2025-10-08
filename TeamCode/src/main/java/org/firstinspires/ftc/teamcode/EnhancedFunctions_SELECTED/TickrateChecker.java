package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

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
}
