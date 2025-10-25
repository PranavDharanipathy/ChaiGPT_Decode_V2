package org.firstinspires.ftc.teamcode.ShooterSystems;

public class Goal {

    @SuppressWarnings("all")
    public enum Tag {

        RED(24),
        BLUE(20);

        private int tagID;

        Tag(int tagID) {
            this.tagID = tagID;
        }

        public int getTagID() {
            return tagID;
        }
    }

    @SuppressWarnings("all")
    public enum Coordinates {

        RED(54.7498,62.5),
        BLUE(-54.7498,62.5);

        private double x;
        private double y;

        Coordinates(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double[] getCoordinates() {
            return new double[] {x, y};
        }
    }



}
