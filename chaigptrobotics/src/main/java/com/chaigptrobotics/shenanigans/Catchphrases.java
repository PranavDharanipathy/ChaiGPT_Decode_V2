package com.chaigptrobotics.shenanigans;

public enum Catchphrases {

    AUTONOMOUS_INIT("Strong auraful Robot, bouta dominate the match right?"),
    AUTONOMOUS_START("He da big R, big R for Robot!"),

    TELEOP_INIT("Cold steel, mid processor, powerful Robot.");

    private String catchphrase;

    Catchphrases(String catchphrase) {
        this.catchphrase = catchphrase;
    }

    public String getCatchphrase() {
        return catchphrase;
    }
}
