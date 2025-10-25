package org.firstinspires.ftc.teamcode.ShooterSystems;

public enum PIPELINES {

    OBELISK_PIPELINE(0),
    RED_PIPELINE(1),
    BLUE_PIPELINE(2),
    TEST_PIPELINE(7);

    private int index = 0; // default index

    PIPELINES(int index) {
        this.index = index;
    }

    public int getIndex() {
        return index;
    }
}