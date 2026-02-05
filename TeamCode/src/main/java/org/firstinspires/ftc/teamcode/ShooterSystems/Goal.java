package org.firstinspires.ftc.teamcode.ShooterSystems;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Goal {

    public static class GoalCoordinate {

        private final double x;
        private final double y;

        public GoalCoordinate(double x, double y) {

            this.x = x;
            this.y = y;
        }

        public double getX() { return x; }
        public double getY() { return y; }
    }

    /**
     * x is forward-backward with forward being positive and backward being negative
     * <p>
     * y is left-right with left being positive and right being negative
     * **/
    @SuppressWarnings("all")
    public enum GoalCoordinates {

        //            CLOSE                                  FAR
        RED(new GoalCoordinate(70,-72), new GoalCoordinate(72,-67)),
        BLUE(new GoalCoordinate(72, 72), new GoalCoordinate(72, 67));

        private GoalCoordinate close;
        private GoalCoordinate far;

        GoalCoordinates(GoalCoordinate close, GoalCoordinate far) {

            this.close = close;
            this.far = far;
        }

        /// Sets the current the close and far {@link GoalCoordinate}
        public void setGoalCoordinates(GoalCoordinate close, GoalCoordinate far) {

            this.close = close;
            this.far = far;
        }

        public GoalCoordinate getCloseCoordinate() {
            return close;
        }
        public GoalCoordinate getFarCoordinate() {
            return far;
        }
    }

    public enum GoalCoordinatesForDistance {

        RED(new GoalCoordinate(60, -60)),
        BLUE(new GoalCoordinate(60, 60));

        private GoalCoordinate coord;

        GoalCoordinatesForDistance(GoalCoordinate coord) {
            this.coord = coord;
        }

        public GoalCoordinate getCoordinate() {
            return coord;
        }
    }

    /// The angle in degrees that is required for any system to look in to be pointing at the goal.
    /// <p>
    /// x is forward-backward and y is left-right.
    /// @param x In inches
    /// @param y In inches
    public static double getAngleToGoal(double x, double y, GoalCoordinate goalCoordinate) {

        double dx = goalCoordinate.getX() - x;
        double dy = goalCoordinate.getY() - y;

        return Math.toDegrees(FastMath.atan2(dy, dx));
    }

    /// Gets the flat (2d) distance from the goal.
    public static double getDistanceFromGoal(double x, double y, GoalCoordinate goalCoordinate) {
        return MathUtil.getDistance2d(x, goalCoordinate.getX(), y, goalCoordinate.getY());
    }

}
