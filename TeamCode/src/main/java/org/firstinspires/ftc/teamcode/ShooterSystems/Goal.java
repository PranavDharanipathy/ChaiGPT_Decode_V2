package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.pedropathing.geometry.Pose;

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
        RED(new GoalCoordinate(69,-78), new GoalCoordinate(40,-72), new GoalCoordinate(68,-72)),
        BLUE(new GoalCoordinate(73, 75), new GoalCoordinate(54, 72), new GoalCoordinate(77, 73));

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

        public GoalCoordinate getCloseCoordinate(double y, GoalCoordinates allianceUsingGoalCoordinates) {

            boolean isOpponent = allianceUsingGoalCoordinates == BLUE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent ? closeOpponent : closeAlliance;
        }

        public GoalCoordinate getCloseCoordinate(double y, CurrentAlliance.ALLIANCE alliance) {

            boolean isOpponent = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent ? closeOpponent : closeAlliance;
        }

        public GoalCoordinate getFarCoordinate() {
            return far;
        }

        // (lateral) y value after which (once y is greater) close goal coordinate switches from alliance to opponent
        public static double RED_CLOSE_GOAL_COORDINATE_SWITCH = -25;
        public static double BLUE_CLOSE_GOAL_COORDINATE_SWITCH = 25;

        public void setRedCloseGoalCoordinateSwitch(double redCloseGoalCoordinateSwitch) {
            RED_CLOSE_GOAL_COORDINATE_SWITCH = redCloseGoalCoordinateSwitch;
        }

        public void setBlueCloseGoalCoordinateSwitch(double blueCloseGoalCoordinateSwitch) {
            BLUE_CLOSE_GOAL_COORDINATE_SWITCH = blueCloseGoalCoordinateSwitch;
        }

        public static boolean onAllianceSide(double y, CurrentAlliance.ALLIANCE alliance) {

            boolean isAlliance = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? y > RED_CLOSE_GOAL_COORDINATE_SWITCH : y < BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isAlliance;
        }

        public boolean onAllianceSide(double y) {

            boolean isAlliance = this == BLUE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isAlliance;
        }

        public static boolean onOpponentSide(double y, CurrentAlliance.ALLIANCE alliance) {

            boolean isOpponent = alliance == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent;
        }

        public boolean onOpponentSide(double y) {

            boolean isOpponent = this == BLUE ? y < RED_CLOSE_GOAL_COORDINATE_SWITCH : y > BLUE_CLOSE_GOAL_COORDINATE_SWITCH;

            return isOpponent;
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

    public static class AprilTagCoordinate {

        private double x, y, z, yaw;

        public AprilTagCoordinate(double x, double y, double z, double yaw) {

            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getZ() {
            return z;
        }

        public double getYaw() {
            return yaw;
        }
    }

    public enum AprilTagCoordinates {

        RED(new AprilTagCoordinate(-56,29.1339,57.5, Math.toRadians(32.3226))),
        BLUE(new AprilTagCoordinate(56,29.1339,57.5, Math.toRadians(32.3226)));

        private AprilTagCoordinate coord;

        AprilTagCoordinates(AprilTagCoordinate coord) {
            this.coord = coord;
        }

        public AprilTagCoordinate getAprilTagCoordinate() {
            return coord;
        }

        public Pose getAsPedroPose() {
            return new Pose(coord.getX(), coord.getZ(), coord.getYaw());
        }

        public static Pose toPedroPose(AprilTagCoordinate aprilTagCoordinate) {
            return new Pose(aprilTagCoordinate.getX(), aprilTagCoordinate.getZ(), aprilTagCoordinate.getYaw());
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
