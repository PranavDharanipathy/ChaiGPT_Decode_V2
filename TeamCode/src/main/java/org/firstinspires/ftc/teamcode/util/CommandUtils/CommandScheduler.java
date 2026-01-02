package org.firstinspires.ftc.teamcode.util.CommandUtils;

import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;

public class CommandScheduler {

    private CommandScheduler() {}

    private static ArrayList<Command> tasks = new ArrayList<>();
    private static ArrayList<T> times = new ArrayList<>();

    /// Schedules command which is automatically run at the appropriate time as long as update() is run.
    public static void schedule(T t, Command task) {

        if (!tasks.contains(task)) {

            tasks.add(task);

            T time = new T(MathUtil.truncate(t.getTime() + getElapsedTime(), 100));
            times.add(time);
        }
    }

    /// Schedules command which is automatically run at the appropriate time as long as update() is run.
    public static void schedule(Command task) {

        if (!tasks.contains(task)) {

            tasks.add(task);

            times.add(new T(0));
        }
    }

    /// In milliseconds
    private static double getElapsedTime() {
        return System.nanoTime() * 1e-6 - startTime;
    }

    private static double startTime = 0;

    public static void start() {
        startTime = getElapsedTime();
    }

    public static void update() {

        for (int index = tasks.size() - 1; index >= 0; index--) {

            if (getElapsedTime() >= times.get(index).getTime()) {

                boolean notComplete = tasks.get(index).run();

                if (!notComplete) {

                    tasks.remove(index);
                    times.remove(index);
                }
            }
        }

    }

    /// Instantly cancels the inputted task.
    public static void cancel(Command task) {

        int index = tasks.indexOf(task);

        if (index != -1) {

            tasks.remove(index);
            times.remove(index);
        }
    }
}
