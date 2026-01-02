package org.firstinspires.ftc.teamcode.util.CommandUtils;

public class CommandStateBoolean {

    private boolean state;

    public CommandStateBoolean(boolean state) {
        this.state = state;
    }

    public void setState(boolean state) {
        this.state = state;
    }

    public boolean getState() {
        return state;
    }
}
