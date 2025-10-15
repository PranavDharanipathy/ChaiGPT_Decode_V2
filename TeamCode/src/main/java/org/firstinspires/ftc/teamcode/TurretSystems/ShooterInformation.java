package org.firstinspires.ftc.teamcode.TurretSystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class ShooterInformation {

    /// All math calculations relevant to the shooter are done in this class
    public strictfp class Calculator extends ShooterInformation {

        public double getTotalFlywheelAssemblyWeight() {
            return ShooterConstants.BASE_FLYWHEEL_ASSEMBLY_WEIGHT + (ShooterConstants.MOI_DISC_WEIGHT * ShooterConstants.NUMBER_OF_MOI_DISCS);
        }

        /// Calculates the height in inches at which the artifact exists the turret
        public double getPreferredShootHeight() {
            return 0;
        }

        /// Calculates the height in inches at which the artifact exists the turret
        public double getPreferredShootAngle() {
            return 0;
        }

        /// Calculates the horizontal distance from the goal in two different ways
        /// Can overcome the slight inaccuracies from the acquiring of data from the camera
        public double[] getHorizontalDistancesFromGoal(double angle, double distanceToGoalFromAngle) {

            distanceToGoalFromAngle += CameraConstants.CAMERA_ANGLE;

            return new double[] {
                    Math.pow(distanceToGoalFromAngle, 2) - Math.pow(Constants.HEIGHT_OF_GOAL_APRIL_TAG, 2), //using pythagorean
                    Math.sin(Math.toRadians(angle)) * distanceToGoalFromAngle //using trigonometry
            };
        }

    }

    /// All Limelight3A camera constants
    public static class CameraConstants extends ShooterInformation {

        /// Camera angle in degrees
        public static double CAMERA_ANGLE = 30;
        /// Camera height in inches
        public static double CAMERA_HEIGHT = 6;
    }

    /// All shooter constants
    public static class ShooterConstants {

        /// The distance in inches from the end of the turret to the pivot point of the turret
        public static double TURRET_LENGTH;

        /// The radius in inches of the gear responsible for moving the hood and thus aiming the turret
        public static double TURRET_AIMER_GEAR_RADIUS;

        /// The angle in degrees that the turret is at when the hood is retracted all the way to the start position
        public static double DEFAULT_TURRET_ANGLE;
        /// The value that makes the servos put the hood at the start position
        public static double DEFAULT_TURRET_HOOD_ANGLER_START_POSITION;

        /// Amount of degrees turret moves per degree that the hood angler moves
        public static double TURRET_DEGREES_PER_HOOD_ANGLER_DEGREE = 48.0 / 392.0;

        /// Positional increment per degree moved by the hood angler servos
        public static double HOOD_ANGLER_POSITIONAL_INCREMENT_PER_DEGREE = 1.0 / 255.0;

        /// Weight of the entire shooter (turret) in grams
        public static double TURRET_WEIGHT = 2779;

        /// Weight of the flywheel assembly in grams
        public static double BASE_FLYWHEEL_ASSEMBLY_WEIGHT = 334;

        /// Weight of an moment-of-inertia disc that goes on the flywheel in grams
        public static double MOI_DISC_WEIGHT = 34;

        public static int NUMBER_OF_MOI_DISCS = 0;

        /// Min and max limits for hood angler
        public static double HOOD_ANGLER_MIN_POSITION_LIMIT = 12;
        public static double HOOD_ANGLER_MAX_POSITION_LIMIT = 220;

        /// Min and max integral limits for turret base
        public static double TURRET_BASE_MIN_INTEGRAL_LIMIT;
        public static double TURRET_BASE_MAX_INTEGRAL_LIMIT;

        /// Min and max integral limits for flywheel
        public static double FLYWHEEL_MIN_INTEGRAL_LIMIT;
        public static double FLYWHEEL_MAX_INTEGRAL_LIMIT;
    }

}