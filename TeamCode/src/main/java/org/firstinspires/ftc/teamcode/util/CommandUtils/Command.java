package org.firstinspires.ftc.teamcode.util.CommandUtils;

public interface Command {

    /// @return 'false' when task is complete, and 'true' when task is not complete
    public boolean run(); //'public' keyword is added for auto-complete
}
