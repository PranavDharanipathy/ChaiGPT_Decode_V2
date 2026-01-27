package org.firstinspires.ftc.teamcode.util.CommandUtils;

@FunctionalInterface
public interface Command {

    /// @return 'false' when task is complete, and 'true' when task is not complete
    public boolean run();
}
