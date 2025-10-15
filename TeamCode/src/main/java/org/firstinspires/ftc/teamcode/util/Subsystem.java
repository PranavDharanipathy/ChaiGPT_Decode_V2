package org.firstinspires.ftc.teamcode.util;

/// Parent class of subsystems
public abstract class Subsystem implements SubsystemInternal {

    protected void provideComponents() {}

    public abstract void update();
}
