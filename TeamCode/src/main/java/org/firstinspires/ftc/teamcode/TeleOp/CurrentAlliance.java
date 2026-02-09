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

    public String toString() {

        String string = "INVALID";

        if (alliance == ALLIANCE.BLUE_ALLIANCE) {
            string = "BLUE";
        }
        else if (alliance == ALLIANCE.RED_ALLIANCE) {
            string = "RED";
        }

        return string;
    }
}