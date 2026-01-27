package org.firstinspires.ftc.teamcode.TeleOp;

public class CurrentAlliance {

    public enum ALLIANCE {
        BLUE_ALLIANCE, RED_ALLIANCE
    }

    private final ALLIANCE alliance;

    public CurrentAlliance(ALLIANCE startAlliance) {
        alliance = startAlliance;
    }

    public ALLIANCE getAlliance() {
        return alliance;
    }
}