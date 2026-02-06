package org.firstinspires.ftc.teamcode.ShooterSystems;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.TeleOp.CurrentAlliance;
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

        //           CLOSE ALLIANCE                  CLOSE OPPONENT                           FAR
        RED(new GoalCoordinate(65,-72), new GoalCoordinate(40,-72), new GoalCoordinate(67,-74)),
        BLUE(new GoalCoordinate(75, 72), new GoalCoordinate(55, 72), new GoalCoordinate(72, 67));

        private GoalCoordinate closeAlliance;
        private GoalCoordinate closeOpponent;
        private GoalCoordinate far;

        GoalCoordinates(GoalCoordinate closeAlliance, GoalCoordinate closeOpponent, GoalCoordinate far) {

            this.closeAlliance = closeAlliance;
            this.closeOpponent = closeOpponent;
            this.far = far;
        }

        /// Sets the current the close and far {@link GoalCoordinate}
        public void setGoalCoordinates(GoalCoordinate closeAlliance, GoalCoordinate closeOpponent, GoalCoordinate far) {

            this.closeAlliance = closeAlliance;
            this.closeOpponent = closeOpponent;
            this.far = far;
        }

        public GoalCoordinate getCloseAllianceCoordinate() {
            return closeAlliance;
        }
        public GoalCoordinate getCloseOpponentCoordinate() {
            return closeOpponent;
        }

        public GoalCoordinate getCloseCoordinate(double turretY, GoalCoordinates allianceUsingGoalCoordinates) {

            boolean isClose = allianceUsingGoalCoordinates == RED ? turretY < RED_CLOSE_GOAL_COORDINATE_SWITCH : turretY > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isClose ? closeAlliance : closeOpponent;
        }

        public GoalCoordinate getCloseCoordinate(double turretY, CurrentAlliance.ALLIANCE alliance) {

                boolean isClose = alliance == CurrentAlliance.ALLIANCE.RED_ALLIANCE ? turretY < RED_CLOSE_GOAL_COORDINATE_SWITCH : turretY > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

                return isClose ? closeAlliance : closeOpponent;
        }

        public GoalCoordinate getFarCoordinate() {
            return far;
        }

        // (lateral) y value after which (once y is greater) close goal coordinate switches from alliance to opponent
        public static double RED_CLOSE_GOAL_COORDINATE_SWITCH = -10;
        public static double BLUE_CLOSE_GOAL_COORDINATE_SWITCH = 10;

        public void setRedCloseGoalCoordinateSwitch(double redCloseGoalCoordinateSwitch) {
            RED_CLOSE_GOAL_COORDINATE_SWITCH = redCloseGoalCoordinateSwitch;
        }

        public void setBlueCloseGoalCoordinateSwitch(double blueCloseGoalCoordinateSwitch) {
            BLUE_CLOSE_GOAL_COORDINATE_SWITCH = blueCloseGoalCoordinateSwitch;
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
