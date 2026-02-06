package org.firstinspires.ftc.teamcode.TeleOp;

public class Obelisk {

    /**
     * <p>GPP: 21
     * <p>PGP: 22
     * <p>PPG: 23
     * <p>INVALID: -1
     **/
    public enum MOTIF {

        GPP(21),
        PGP(22),
        PPG(23),
        INVALID(-1);

        private int motifAprilTagNumber;

        MOTIF(int aprilTagNumber) {
            motifAprilTagNumber = aprilTagNumber;
        }

        public synchronized int getAprilTagNumber() {
            return motifAprilTagNumber;
        }
    }

    private MOTIF motif;

    public Obelisk(MOTIF startMotif) {
        motif = startMotif;
    }

    public void setMotif(MOTIF motif) {
        this.motif = motif;
    }

    public MOTIF getMotif() {
        return motif;
    }

    public static MOTIF getFromAprilTagNumber(int aprilTagNumber) {

        MOTIF motif;

        switch (aprilTagNumber) {

            case 21:
                motif = MOTIF.GPP;
                break;

            case 22:
                motif = MOTIF.PGP;
                break;

            case 23:
                motif = MOTIF.PPG;
                break;

            default:
                throw new IllegalArgumentException("aprilTagNumber can only be 21, 22 or 23!");
        }

        return motif;
    }
}
