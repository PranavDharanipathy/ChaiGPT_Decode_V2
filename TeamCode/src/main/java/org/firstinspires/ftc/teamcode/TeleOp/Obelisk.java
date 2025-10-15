package org.firstinspires.ftc.teamcode.TeleOp;

public class Obelisk {

    /**
     * <p>GPP: 21
     * <p>PGP: 22
     * <p>PPG: 23
     * <p>INVALID: -1
     **/
    public enum OBELISK {

        GPP(21),
        PGP(22),
        PPG(23),
        INVALID(-1);

        private int obeliskModeAprilTagNumber;

        OBELISK(int aprilTagNumber) {
            obeliskModeAprilTagNumber = aprilTagNumber;
        }

        public synchronized int getAprilTagNumber() {
            return obeliskModeAprilTagNumber;
        }
    }

    private Obelisk.OBELISK obelisk;

    public Obelisk(Obelisk.OBELISK startObelisk) {
        obelisk = startObelisk;
    }

    public void setObelisk(Obelisk.OBELISK obelisk) {
        this.obelisk = obelisk;
    }

    public Obelisk.OBELISK getObelisk() {
        return obelisk;
    }

    public static OBELISK getFromAprilTagNumber(int aprilTagNumber) {

        OBELISK obelisk;

        switch (aprilTagNumber) {

            case 21:
                obelisk = OBELISK.GPP;
                break;

            case 22:
                obelisk = OBELISK.PGP;
                break;

            case 23:
                obelisk = OBELISK.PPG;
                break;

            default:
                throw new IllegalArgumentException("aprilTagNumber can only be 21, 22 or 23!");
        }

        return obelisk;
    }
}
