package org.firstinspires.ftc.teamcode.ShooterSystems;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Goal {

    /**
     * x is forward-backward with forward being positive and backward being negative
     * <p>
     * y is left-right with left being positive and right being negative
     * **/
    @SuppressWarnings("all")
    public enum GoalCoordinates {

        RED(58,-60),
        BLUE(60, 60);

        private double x;
        private double y;

        GoalCoordinates(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double[] getCoordinate() {
            return new double[] {x, y};
        }
    }

    /// The angle in degrees that is required for any system to look in to be pointing at the goal.
    /// <p>
    /// x is forward-backward and y is left-right.
    /// @param x In inches
    /// @param y In inches
    public static double getAngleToGoal(double x, double y, GoalCoordinates goalCoordinates) {

        double dx = goalCoordinates.x - x;
        double dy = goalCoordinates.y - y;

        return Math.toDegrees(FastMath.atan2(dy, dx));
    }

    /// Gets the flat (2d) distance from the goal.
    public static double getDistanceFromGoal(double x, double y, GoalCoordinates goalCoordinates) {
        return MathUtil.getDistance2d(x, goalCoordinates.x, y, goalCoordinates.y);
    }

}
