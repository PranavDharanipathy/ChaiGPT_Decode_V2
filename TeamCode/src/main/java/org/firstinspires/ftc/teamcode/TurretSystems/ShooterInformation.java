package org.firstinspires.ftc.teamcode.TurretSystems;

public class ShooterInformation {

    /// All math calculations relevant to the shooter are done in this class
    public class Calculator extends ShooterInformation {

        /// Calculates the height in inches at which the artifact exists the turret
        public double getCurrentShootHeight() {
            return 0;
        }

        /// Calculates the height in inches at which the artifact exists the turret
        public double getCurrentShootAngle() {
            return 0;
        }

    }

    /// All Limelight3A camera constants
    public class CameraConstants extends ShooterInformation {

        /// Camera angle in degrees
        public double CAMERA_ANGLE = 30;
        /// Camera height in inches
        public double CAMERA_HEIGHT = 6;
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
        public static double TURRET_DEGREES_PER_HOOD_ANGLER_DEGREE = 1;

        /// Positional increment per degree moved by the hood angler servos
        public static double HOOD_ANGLER_POSITIONAL_INCREMENT_PER_DEGREE;
    }

}