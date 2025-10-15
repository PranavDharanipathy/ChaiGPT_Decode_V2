package org.firstinspires.ftc.teamcode.TeleOp;

public class CurrentAlliance {

    public enum ALLIANCE {
        BLUE_ALLIANCE, RED_ALLIANCE
    }

    private ALLIANCE alliance;

    public CurrentAlliance(ALLIANCE startAlliance) {
        alliance = startAlliance;
    }

    public void setAlliance(ALLIANCE alliance) {
        this.alliance = alliance;
    }

    public ALLIANCE getAlliance() {
        return alliance;
    }
}