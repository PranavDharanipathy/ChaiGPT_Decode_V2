package org.firstinspires.ftc.teamcode.Auton;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

// -------------------- COMMAND INTERFACE --------------------
interface Command {
    void checkTriggered(boolean commandTrigger);
    void runInstance();
    boolean isFinished();
}

// -------------------- WAIT COMMAND --------------------
class WaitCommand implements Command {
    private final long duration;
    private long startTime = 0L;
    private boolean started = false;

    public WaitCommand(long milliseconds) {
        this.duration = milliseconds;
    }

    @Override
    public void checkTriggered(boolean commandTrigger) {
        // no-op
    }

    @Override
    public void runInstance() {
        if (!started) {
            startTime = System.currentTimeMillis();
            started = true;
        }
    }

    @Override
    public boolean isFinished() {
        return started && (System.currentTimeMillis() - startTime >= duration);
    }
}

// -------------------- COMMAND SCHEDULER --------------------
class CommandScheduler {
    private final List<Command> activeCommands = new ArrayList<>();

    public void add(Command command) {
        if (command != null) activeCommands.add(command);
    }


    //Do I need this code?
   /* public void run() {
        Iterator<Command> it = activeCommands.iterator();
        while (it.hasNext()) {
            Command c = it.next();
            c.runInstance();
            if (c.isFinished()) {
                it.remove();
            }
        }
    } */

}
