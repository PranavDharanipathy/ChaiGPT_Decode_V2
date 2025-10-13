package org.firstinspires.ftc.teamcode.util;

public class CurrentAlliance {

    public enum ALLIANCE {
        BLUE_ALLIANCE, RED_ALLIANCE;
    }

    private ALLIANCE alliance;

    private CurrentAlliance(ALLIANCE startAlliance) {
        alliance = startAlliance;
    }

    public CurrentAlliance setStartAlliance(ALLIANCE startAlliance) {
        return new CurrentAlliance(startAlliance);
    }

    public void setAlliance(ALLIANCE alliance) {
        this.alliance = alliance;
    }

    public ALLIANCE getAlliance() {
        return alliance;
    }
}