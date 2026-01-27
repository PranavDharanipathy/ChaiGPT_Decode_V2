package org.firstinspires.ftc.teamcode.util;

public class BooleanTrigger {

    private boolean trigger;

    public BooleanTrigger(boolean trigger) {
        this.trigger = trigger;
    }

    public void set(boolean trigger) {
        this.trigger = trigger;
    }

    public void and(BooleanTrigger booleanTrigger) {
        trigger = booleanTrigger.get() && trigger;
    }

    public void or(BooleanTrigger booleanTrigger) {
        trigger = booleanTrigger.get() || trigger;
    }

    public void not(BooleanTrigger booleanTrigger) {
        trigger = !booleanTrigger.get();
    }

    public boolean get() {
        return trigger;
    }
}
