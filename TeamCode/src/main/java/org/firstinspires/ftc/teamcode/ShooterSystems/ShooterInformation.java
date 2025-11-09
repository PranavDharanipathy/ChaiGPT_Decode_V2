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

        /// The radius in inches of the gear responsible for moving the hood and thus aiming the turret
        public static double TURRET_GEAR_RADIUS;

        /// Number of ticks the REV Through-Bore encoder needs to turn to turn 1 degree
        public static double TURRET_ENCODER_DEGREE_MULTIPLIER = 8192.0 / 360.0;

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

        public static double TURRET_CLOSE_MULTIPLIER = 1;
        public static double TURRET_FAR_MULTIPLIER = 0.9;

        public static double HOOD_POSITION_MANUAL_INCREMENT = 0.035;

        public static double HOOD_CLOSE_POSITION = 0.51;
        public static double HOOD_FAR_POSITION = 0.19;

        public static double TURRET_POSITIONAL_OFFSET = -2.231;
        public static double TURRET_ANGULAR_OFFSET = 180;
    }

    public static class Calculator {

        public static double convert2dGoalDistanceTo3dToAprilTag(double flatDistanceFromGoal) {
            return Math.sqrt(Math.pow(flatDistanceFromGoal, 2) + Math.pow(Constants.HEIGHT_OF_GOAL_APRIL_TAG, 2));
        }

        public static double convert2dGoalDistanceTo3dToGoal(double flatDistanceFromGoal) {
            return Math.sqrt(Math.pow(flatDistanceFromGoal, 2) + Math.pow(Constants.HEIGHT_OF_GOAL, 2));
        }

        public static double robotOffsetX = 0;
        public static double robotOffsetY = 0;
        public static double robotOffsetHeading = 0;

        public static void setBotPoseReZeroingOffsets(double offsetX, double offsetY, double offsetHeading) {

            robotOffsetX = offsetX;
            robotOffsetY = offsetY;
            robotOffsetHeading = offsetHeading;
        }

        /// Field-relative
        /// <p>
        /// Gets an x, y, and heading offset that are to be added to the robot pose to help us determine the
        /// robot's coordinates and heading where the center of the field and pointing forward is (0,0,0).
        /// <p>
        /// The driver drives to the reZeroPose after which when this calculation is run it creates offsets
        /// to offset the current the known pose which the driver has driven to.
        public static void calculateBotPoseReZeroingOffsets(Pose2d robotPose, Pose2d reZeroPose) {

            //pose what the pose should be
            Vector2d reZeroPosition = reZeroPose.position;
            double reZeroHeading = Math.toDegrees(reZeroPose.heading.toDouble());

            Vector2d rawPosition = robotPose.position;

            double rawHeading = Math.toDegrees(robotPose.heading.toDouble());

            double robotOffsetX = reZeroPosition.x - rawPosition.x;
            double robotOffsetY = reZeroPosition.y - rawPosition.y;

            double robotOffsetHeading = reZeroHeading - rawHeading;
        }

        /// Gets the re-zeroed bot pose.
        public static Pose2d getBotPose(Pose2d robotPose) {

            Vector2d position = robotPose.position;
            double heading = Math.toDegrees(robotPose.heading.toDouble());

            return new Pose2d(
                    position.x + robotOffsetX,
                    position.y + robotOffsetY,
                    heading + robotOffsetHeading
            );
        }

        /// Field-relative
        public static Pose2d getTurretPoseFromBotPose(Pose2d reZeroedRobotPose, double reZeroedTurretTicks) {

            Vector2d robotPosition = reZeroedRobotPose.position;

            double robotHeading = Math.toDegrees(reZeroedRobotPose.heading.toDouble());

            double turretX = robotPosition.x + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.cos(robotHeading));
            double turretY = robotPosition.y + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.sin(robotHeading));

            double turretHeading = robotHeading + ShooterConstants.TURRET_ANGULAR_OFFSET + (reZeroedTurretTicks / ShooterConstants.TURRET_TICKS_PER_DEGREE);

            return new Pose2d(
                    turretX,
                    turretY,
                    turretHeading
            );
        }

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