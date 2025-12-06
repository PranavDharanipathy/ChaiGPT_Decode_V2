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
        RED(new GoalCoordinate(64,-70), new GoalCoordinate(79,-72)),
        BLUE(new GoalCoordinate(66, 66), new GoalCoordinate(79, 58));

        private GoalCoordinate close;
        private GoalCoordinate far;

        GoalCoordinates(GoalCoordinate close, GoalCoordinate far) {

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
