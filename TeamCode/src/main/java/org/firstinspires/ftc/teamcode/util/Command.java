package org.firstinspires.ftc.teamcode.util;

/// Parent class of commands
public interface Command {

    void checkTriggered(boolean commandTrigger);

    void runInstance();
}