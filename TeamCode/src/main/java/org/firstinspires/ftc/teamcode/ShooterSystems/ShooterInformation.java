package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.chaigptrobotics.systems.DeprecatedSystem;
import com.pedropathing.geometry.Pose;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseVelocity;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;
import java.util.List;

public strictfp class ShooterInformation {

    /// All Limelight3A camera constants
    @DeprecatedSystem(notes = "We no longer use the Limelight3A to aim our turret but Odometry instead")
    public static class CameraConstants {

        /// Camera angle in degrees
        public static double CAMERA_ANGLE = 18;
        /// Camera height in inches
        public static double CAMERA_HEIGHT = 11;

        /// Camera poll Hz rate
        public static int CAMERA_POLL_RATE = 50;

        /// Distance in inches
        public static double CAMERA_TO_POINT_OF_ROTATION_2D = 6.28085;

    }

    /// All shooter constants
    public static class ShooterConstants {

        /// Point that differentiates the turret pointing at the goal far/close position.
        public static double FAR_ZONE_CLOSE_ZONE_BARRIER = -35;

        /// Weight of the entire shooter (turret) in grams
        public static double TURRET_WEIGHT = 2779;

        /// Weight of the flywheel assembly in grams
        public static double BASE_FLYWHEEL_ASSEMBLY_WEIGHT = 334;

        /// Weight of a moment-of-inertia disc that goes on the flywheel in grams
        public static double MOI_DISC_WEIGHT = 150;

        public static int NUMBER_OF_MOI_DISCS = 8;

        /// Shaft diameter in millimeter
        public static double SHAFT_DIAMETER = 8;

        public static double FLYWHEEL_MOTOR_CORE_VOLTAGE = 12;
        public static double FLYWHEEL_MOTOR_RPM = 6000;

        /// Min and max limits for hood angler
        public static double HOOD_ANGLER_MIN_POSITION = 0.91;
        public static double HOOD_ANGLER_MAX_POSITION = 0.0;

        public static double getTotalFlywheelAssemblyWeight() {
            return ShooterConstants.BASE_FLYWHEEL_ASSEMBLY_WEIGHT + (ShooterConstants.MOI_DISC_WEIGHT * ShooterConstants.NUMBER_OF_MOI_DISCS);
        }

        public static double getTotalShooterAssemblyWeight() {
            return getTotalFlywheelAssemblyWeight() + ShooterConstants.TURRET_WEIGHT;
        }

        public static double FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY = 408_000;

        public static double CLOSER_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY = 340_000;
        public static double FARTHER_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY = 345_000;
        public static double OPPONENT_SIDE_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY = 375_000;

        /** Distance to goal when shooting at close where flywheel velocity switches from farther close to closer close when
        bot is within this distance to the goal. */
        public static double CLOSE_SIDE_SWITCH = 64;

        public static double TURRET_TARGET_POSITION_ERROR_MARGIN = 50;
        public static int NORMAL_CONTROLLER_RUMBLE_TIME = 300;

        //normalized
        public static double MIN_TURRET_POSITION_IN_DEGREES = -135, MAX_TURRET_POSITION_IN_DEGREES = 135;

        public static double TURRET_TICKS_PER_DEGREE = 73.5179487179; //it should include the turret gear ratio -> (encoder rotations per turret rotation) * (8192 / 360)
        public static double TURRET_DEADBAND_TICKS = 0.013 * TURRET_TICKS_PER_DEGREE;

        public static List<Double> TURRET_PD_POSITIONS = new ArrayList<>(List.of(-11000.0,  -10000.0,    -9000.0,   -8000.0,   -7000.0,   -6000.0,    -5000.0,    -4000.0,    -3000.0,   -2000.0,  -1000.0,     0.0,      1000.0,    2000.0,    3000.0,   4000.0,   5000.0,   6000.0,    7000.0,    8000.0,   9000.0,   10000.0,   11000.0,    12000.0,   13000.0));
        public static List<Double> TURRET_KPS =          new ArrayList<>(List.of(0.000285,   0.000279,   0.000285,   0.0002,    0.0002,   0.00014,    0.00011,    0.00009,    0.00008,   0.00007,  0.00005,   0.00005,   0.000092,  0.0000975,  0.0001,   0.00011,  0.00012,  0.00012,  0.00012,    0.00011,  0.00011,  0.0001,    0.00014,    0.0002,    0.00033));
        public static List<Double> TURRET_KDS =          new ArrayList<>(List.of(  0.003,    0.00318,     0.003,      0.003,    0.003,     0.0028,     0.0024,     0.0022,     0.0022,    0.002,    0.002,     0.002,     0.00385,   0.00375,   0.0032,   0.00353,  0.0035,   0.0032,    0.0032,     0.003,    0.003,   0.0035,    0.0031,     0.0025,     0.002));

        public static List<Double> TURRET_FEEDFORWARD_POSITIONS = new ArrayList<>(List.of(-110000.0, -10000.0,    -9000.0,   -8000.0,    -7000.0,    -6000.0,    -5000.0,     -4000.0,    -3000.0,   -2000.0,    -1000.0,     0.0,       1000.0,    2000.0,    3000.0,     4000.0,     5000.0,   6000.0,     7000.0,     8000.0,   9000.0,    10000.0,   11000.0,    12000.0,   13000.0));
        public static List<Double> TURRET_KFS =                   new ArrayList<>(List.of( 0.000003, 0.0000021,  0.0000021,  0.000002,  0.00000175, 0.00000185,  0.0000018,  0.00000185, 0.0000025, 0.0000025,   0.000003,  0.000011,   0.000008,  0.000007,  0.0000032,   0.000003,  0.000003, 0.000003,  0.0000045,  0.0000045, 0.000003,  0.00000305, 0.0000035,  0.000004,  0.000006));

        public static double TURRET_KF_RESISTANCE_ENGAGE_ERROR = 2000;

        public static double TURRET_HOME_POSITION_INCREMENT = 150;

        public static double HOOD_POSITION_MANUAL_INCREMENT = 0.05;

        public static double HOOD_CLOSE_POSITION = 0.25;
        public static double HOOD_FAR_POSITION = 0.16;

        public static double TURRET_POSITIONAL_OFFSET = -2.231;
        public static double TURRET_ANGULAR_OFFSET = 180;

        /*
         * the TURRET_ANGULAR_OFFSET is multiplied by the correct multiplier to make
         * the turret spin the proper way.
         */
        public static int BLUE_TURRET_ANGULAR_OFFSET_DIRECTION = 1;
        public static int RED_TURRET_ANGULAR_OFFSET_DIRECTION = -1;

        /// The robot velocities must be greater than this for turret hysteresis control to be used.
        /// <p>
        /// Index 0 is translational, index 1 in angular (in radians).
        /// <p>
        /// Translational is in inches per second and angular is in radians per second.
        public static double[] TURRET_HYSTERESIS_CONTROL_ENGAGE_VELOCITY = {10, Math.toRadians(15)};
    }

    public static class Calculator {

        /// Field-relative
        public static Pose getTurretPoseFromBotPose(Pose botPose, double turretPositionTicks, double turretStartPositionTicks) {

            double reZeroedTurretTicks = turretPositionTicks - turretStartPositionTicks;
            double turretRotation = Math.toRadians(reZeroedTurretTicks / ShooterConstants.TURRET_TICKS_PER_DEGREE);

            double turretHeading = botPose.getHeading() + turretRotation + Math.toRadians(ShooterConstants.TURRET_ANGULAR_OFFSET);

            double turretX = botPose.getX() + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.cos(botPose.getHeading()));
            double turretY = botPose.getY() + (ShooterConstants.TURRET_POSITIONAL_OFFSET * FastMath.sin(botPose.getHeading()));

            return new Pose(
                    turretX,
                    turretY,
                    turretHeading
            );
        }

        public static Pose getFutureRobotPose(double t, Pose currentRobotPose, PoseVelocity poseVelocity) {

            return new Pose(
                    currentRobotPose.getX() + (t * poseVelocity.getXVelocity()),
                    currentRobotPose.getY() + (t * poseVelocity.getYVelocity()),
                    currentRobotPose.getHeading() + (t * poseVelocity.getAngularVelocity())
            );
        }

        public static double getRobotTranslationalVelocity(double xVelocity, double yVelocity) {
            return Math.hypot(xVelocity, yVelocity);
        }

        /// For turret hysteresis control - the amount of time in the future where the robot's pose
        /// will be predicted based on it's current pose and velocity as well as the turret's acceleration.
        /// @param turretAcceleration is in rad/sec^2
        public static double getTurretFuturePosePredictionTime(double turretAcceleration) {

            final double NORMAL_ACCEL = Math.toRadians(7.2);
            final double NORMAL = 1.5;
            final double MAX = 2.5;

            final double turretAccel = Math.abs(turretAcceleration);

            double futurePredictionTime = NORMAL * (1 + Math.sqrt(NORMAL_ACCEL / (turretAccel + 1e-3)));

            return MathUtil.clamp(futurePredictionTime, NORMAL, MAX);
        }

    }

    public static class Odometry {

        public enum RELOCALIZATION_POSES {

            CENTER(0),
            BACK(1);

            private int index;

            RELOCALIZATION_POSES(int index) {
                this.index = index;
            }

            public int getPoseIndex() {
                return index;
            }

            public Pose toPedroPose() {

                double[] reZeroPose = REZERO_POSES[index];
                return new Pose(reZeroPose[0], reZeroPose[1], Math.toRadians(reZeroPose[2]));
            }
        }

        public static double[][] REZERO_POSES = {
                {0,0,0}, //center-center-forward-pointing
                {-62.5,0,180}, //back-center-backward-pointing
//                {62.5,0,0} //front-center-forward-pointing
        };

        public static Pose convertReZeroPoseToPedro(double[] reZeroPose) {
            return new Pose(reZeroPose[0], reZeroPose[1], Math.toRadians(reZeroPose[2]));
        }
    }

    public static class Models {

        /// Gets the flat (2d) distance from the goal using the limelight 3A's ty value.
        @DeprecatedSystem(notes = "We are now using odometry to get the distance from the goal, not the limelight")
        public static double getDistanceFromRegression(double ty) {
            return (0.34197 * ty * ty) - (3.79725 * ty) + 53.01088;
        }

        public static double getScaledFlywheelKv(double unscaledKv, double currentVoltage) {
            return (12.5 / currentVoltage) * unscaledKv;
        }

        public static double getCloseHoodPositionFromRegression(double distance) {

            double hoodPosition = ShooterConstants.HOOD_CLOSE_POSITION; //default

            final double PIECEWISE_SWITCH_POINT_CLOSER = 51.9629;
            final double PIECEWISE_SWITCH_POINT_FARTHER = 59.64101;

            // 0 means that it's on the smaller-distance side while 1 means that it's on the larger-distance side
            int closerSwitch = distance <= PIECEWISE_SWITCH_POINT_CLOSER ? 0 : 1;
            int fartherSwitch = distance <= PIECEWISE_SWITCH_POINT_FARTHER ? 0 : 1;

            if (distance < ShooterConstants.CLOSE_SIDE_SWITCH) {
                //closer

                switch (closerSwitch) {

                    case 0:
                        hoodPosition = -0.000023057 * FastMath.pow(distance, 3) + 0.0039832 * FastMath.pow(distance,2) - 0.235099 * distance + 4.85929;
                        break;

                    case 1:
                        hoodPosition = 0.0000546434 * FastMath.pow(distance, 3) - 0.0102611 * FastMath.pow(distance,2) + 0.645521 * distance - 13.34069;
                        break;
                }
            }
            else {
                //farther

                switch (fartherSwitch) {

                    case 0:
                        hoodPosition = -0.0000410919 * FastMath.pow(distance, 3) + 0.00595066 * FastMath.pow(distance,2) - 0.287948 * distance + 4.90789;
                        break;

                    case 1:
                        hoodPosition = 0.000001181 * FastMath.pow(distance, 3) - 0.000323005 * FastMath.pow(distance,2) + 0.03401 * distance - 0.946271;
                        break;
                }
            }

            return hoodPosition;

        }
    }

}