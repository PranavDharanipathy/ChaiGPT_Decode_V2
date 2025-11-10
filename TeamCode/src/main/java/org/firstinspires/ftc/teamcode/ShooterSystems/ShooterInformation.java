package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.chaigptrobotics.systems.DeprecatedSystem;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public final class ShooterInformation {

    /// All Limelight3A camera constants
    public static class CameraConstants {

        /// Camera angle in degrees
        public static double CAMERA_ANGLE = 18;
        /// Camera height in inches
        public static double CAMERA_HEIGHT = 11;

        /// Camera poll Hz rate
        public static int CAMERA_POLL_RATE = 90;

        /// Distance in inches
        public static double CAMERA_TO_POINT_OF_ROTATION_2D = 6.28085;

    }

    /// All shooter constants
    public static class ShooterConstants {

        /// Weight of the entire shooter (turret) in grams
        public static double TURRET_WEIGHT = 2779;

        /// Weight of the flywheel assembly in grams
        public static double BASE_FLYWHEEL_ASSEMBLY_WEIGHT = 334;

        /// Weight of an moment-of-inertia disc that goes on the flywheel in grams
        public static double MOI_DISC_WEIGHT = 34;

        public static int NUMBER_OF_MOI_DISCS = 3;

        public static double SHAFT_DIAMETER = 8;

        public static double MOTOR_CORE_VOLTAGE = 12;
        public static double MOTOR_RPM = 6000;
        public static double BURST_DECELERATION_RATE = 400;

        /// Min and max limits for hood angler
        public static double HOOD_ANGLER_MIN_POSITION = 0.9;
        public static double HOOD_ANGLER_MAX_POSITION = 0.11;

        public static double getTotalFlywheelAssemblyWeight() {
            return ShooterConstants.BASE_FLYWHEEL_ASSEMBLY_WEIGHT + (ShooterConstants.MOI_DISC_WEIGHT * ShooterConstants.NUMBER_OF_MOI_DISCS);
        }

        public static double getTotalShooterAssemblyWeight() {
            return getTotalFlywheelAssemblyWeight() + ShooterConstants.TURRET_WEIGHT;
        }

        public static double FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY = 30_000;
        public static double CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY = 26_500;

        public static double FLYWHEEL_SHOOT_VELOCITY_CONTROLLER_RUMBLE_MARGIN = 800;
        public static int NORMAL_CONTROLLER_RUMBLE_TIME = 300;

        //normalized
        public static double MIN_TURRET_POSITION_IN_DEGREES = -135, MAX_TURRET_POSITION_IN_DEGREES = 135;

        public static double TURRET_TICKS_PER_DEGREE = 73.5179487179; //it should include the turret gear ratio -> (encoder rotations per turret rotation) * (8192 / 360)
        public static double TURRET_DEADBAND_TICKS = 0.2 * 73.5179487179;

        public static double HOOD_POSITION_MANUAL_INCREMENT = 0.035;

        public static double HOOD_CLOSE_POSITION = 0.51;
        public static double HOOD_FAR_POSITION = 0.225;

        public static double TURRET_POSITIONAL_OFFSET = -2.231;
        public static double TURRET_ANGULAR_OFFSET = 180;

        /*
         * the TURRET_ANGULAR_OFFSET is multiplier by the correct multiplier to make
         * the turret spin the proper way.
         */
        public static int BLUE_TURRET_ANGULAR_OFFSET_MULTIPLIER = -1;
        public static int RED_TURRET_ANGULAR_OFFSET_MULTIPLIER = 1;
    }

    public static class Calculator {

        public static double convert2dGoalDistanceTo3dToAprilTag(double flatDistanceFromGoal) {
            return Math.sqrt(Math.pow(flatDistanceFromGoal, 2) + Math.pow(Constants.HEIGHT_OF_GOAL_APRIL_TAG, 2));
        }

        public static double convert2dGoalDistanceTo3dToGoal(double flatDistanceFromGoal) {
            return Math.sqrt(Math.pow(flatDistanceFromGoal, 2) + Math.pow(Constants.HEIGHT_OF_GOAL, 2));
        }

        /// @return The normalized bot pose as a {@link Pose2d}
        public static Pose2d getBotPose(Vector2d robotPosition, double headingRad) {

            double heading = Math.toDegrees(headingRad);

            return new Pose2d(
                    robotPosition.x,
                    robotPosition.y,
                    heading
            );
        }

        /// Field-relative
        public static Pose2d getTurretPoseFromBotPose(Vector2d normalizedRobotPosition, double headingRad, double turretPositionTicks, double turretStartPositionTicks) {

            double reZeroedTurretTicks = turretPositionTicks - turretStartPositionTicks;
            double turretRotation = reZeroedTurretTicks / ShooterConstants.TURRET_TICKS_PER_DEGREE;

            double turretHeading = Math.toDegrees(headingRad) + turretRotation + ShooterConstants.TURRET_ANGULAR_OFFSET;

            double turretX = normalizedRobotPosition.x + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.cos(headingRad));
            double turretY = normalizedRobotPosition.y + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.sin(headingRad));

            return new Pose2d(
                    turretX,
                    turretY,
                    turretHeading
            );
        }

    }

    public static class Odometry {

        public static double[][] REZERO_POSES = {
                {0,0,0}, //center-center-forward-pointing
                {-62.5,0,180}, //back-center-backward-pointing
                {62.5,0,0} //front-center-forward-pointing
        };
    }

    public static class Regressions {

        /// Gets the flat (2d) distance from the goal using the limelight 3A's ty value.
        @DeprecatedSystem(notes = "We are now using odometry to get the distance from the goal, not the limelight")
        public static double getDistanceFromRegression(double ty) {
            //quadratic regression
            return (0.34197 * Math.pow(ty, 2)) - (3.79725 * ty) + 53.01088;
        }
    }

}