package org.firstinspires.ftc.teamcode.util.CommandUtils;

public class InstantCommand implements Command {

    private Runnable task;

    public InstantCommand(Runnable task) {
        this.task = task;
    }

    @Override
    public boolean run() {
        task.run();
        return false;
    }
}