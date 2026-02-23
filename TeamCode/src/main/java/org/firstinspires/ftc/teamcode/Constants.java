package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.chaigptrobotics.systems.DeprecatedSystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBasePIDFSCoefficients;
import org.firstinspires.ftc.teamcode.TeleOp.CurrentAlliance;
import org.firstinspires.ftc.teamcode.data.EOAOffset;

import java.util.HashMap;
import java.util.Map;

//@Config
public class Constants {

    public static class DriveConstants {

        public static Pose RED_SHOOT_POSE = new Pose(15, -10);
        public static Pose BLUE_SHOOT_POSE = new Pose(15, 10);
        public static double[] SHOOT_POSE_TOLERANCE = {4, 4};

        public static Pose getShootPose(CurrentAlliance alliance) {
            return alliance.getAlliance() == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? BLUE_SHOOT_POSE : RED_SHOOT_POSE;
        }

        public static PathChain getMoveToShootPathChain(CurrentAlliance alliance, Follower follower, Pose driveToShootOrigPose) {

            Pose shootPose = alliance.getAlliance() == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? BLUE_SHOOT_POSE : RED_SHOOT_POSE;

            double poseYDifference = shootPose.getY() - driveToShootOrigPose.getY();
            HeadingInterpolator headingFunction =
                    Math.abs(poseYDifference) < 20 ||
                    (driveToShootOrigPose.getY() < shootPose.getY() && alliance.getAlliance() == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE) ||
                    (driveToShootOrigPose.getY() > shootPose.getY() && alliance.getAlliance() == CurrentAlliance.ALLIANCE.RED_ALLIANCE)
                            ? HeadingInterpolator.facingPoint(-72, driveToShootOrigPose.getY())
                            : HeadingInterpolator.facingPoint(-72, poseYDifference);

            boolean decelerationAllowed = driveToShootOrigPose.distanceFrom(shootPose) < 30;
            PathChain.DecelerationType deceleration = decelerationAllowed ? PathChain.DecelerationType.GLOBAL : PathChain.DecelerationType.NONE;

            PathChain pathChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    driveToShootOrigPose,
                                    shootPose
                            )
                    )
                    .setHeadingInterpolation(headingFunction)
                    .build();

            pathChain.setDecelerationType(deceleration);

            return pathChain;
        }

        public static Pose RED_BASE_POSE = new Pose(-47.5, 35.85, Math.toRadians(180));
        public static Pose BLUE_BASE_POSE = new Pose(-47.5, -35.85, Math.toRadians(180));
        public static double[] BASE_POSE_TOLERANCE = {1, 1, Math.toRadians(1)};

        public static Pose getBasePose(CurrentAlliance alliance) {
            return alliance.getAlliance() == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? BLUE_BASE_POSE : RED_BASE_POSE;
        }

        public static PathChain getMoveToBasePathChain(CurrentAlliance alliance, Follower follower) {

            Pose basePose = alliance.getAlliance() == CurrentAlliance.ALLIANCE.BLUE_ALLIANCE ? BLUE_BASE_POSE : RED_BASE_POSE;

            return follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    follower.getPose(),
                                    basePose
                            )
                    )
                    .setConstantHeadingInterpolation(basePose.getHeading())
                    .build();
        }
    }

    public static class IMUConstants {

        public static RevHubOrientationOnRobot CONTROL_HUB_ORIENTATION_IMU_PARAMETER =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        public static Rev9AxisImuOrientationOnRobot REV_9_AXIS_IMU_ORIENTATION_IMU_PARAMETER =
                new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD);

        public static Orientation getInternalIMUOrientationStats(@NonNull IMU internalIMU) {

            return internalIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        }

        public static Orientation getRev9AxisIMUOrientationStats(@NonNull Rev9AxisImu rev9AxisIMU) {

            return rev9AxisIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        }

        public static IMU.Parameters getInternalIMUParams() {
            return new IMU.Parameters(CONTROL_HUB_ORIENTATION_IMU_PARAMETER);
        }

        public static Rev9AxisImu.Parameters getRev9AxisIMUParams() {

            Rev9AxisImu.Parameters rev9axisIMUParams = new Rev9AxisImu.Parameters(REV_9_AXIS_IMU_ORIENTATION_IMU_PARAMETER);
            rev9axisIMUParams.calibrationDataFile = "Rev9AxisImuCalibration.json";

            return rev9axisIMUParams;
        }
    }

    public static class MapSetterConstants {


        public static LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

        public static String controlHubDeviceName = "Control Hub";
        public static String expansionHubDeviceName = "Expansion Hub";

        public static String rev9AxisIMUDeviceName = "rev9axisimu";
        public static String internalIMUDeviceName = "imu";

        public static String pinpointOdometryComputerDeviceName = "pinpoint";

        public static String limelight3AUSBDeviceName = "limelight";

        public static String leftFrontMotorDeviceName = "left_front";
        public static String leftBackMotorDeviceName = "left_back";
        public static String rightFrontMotorDeviceName = "right_front";
        public static String rightBackMotorDeviceName = "right_back";

        public static String turretExternalEncoderMotorPairName = "right_back";

        public static String hoodAnglerLeftServoDeviceName = "left_hood_angler";
        public static String hoodAnglerRightServoDeviceName = "right_hood_angler";

        public static String leftFlywheelMotorDeviceName = "left_flywheel";
        public static String rightFlywheelMotorDeviceName = "right_flywheel";

        public static String turretBaseLeftServoDeviceName = "left_turret_base";
        public static String turretBaseRightServoDeviceName = "right_turret_base";

        public static String intakeMotorDeviceName = "intake";
        public static String liftPTOServoDeviceName = "lift_pto";

        public static String transferMotorDeviceName = "transfer";

        public static String[] transferBeambreakSensorNames = {"transfer_beambreak_power", "transfer_beambreak_receiver"};

        public static String[] intakeBeambreakSensorNames = {"intake_beambreak_power", "intake_beambreak_receiver"};

        public static String[] rev2mDistanceSensorNames = {"left_distance_sensor", "back_distance_sensor", "right_distance_sensor"};
    }

    public static class IOConstants {

        /// End-of-auto (EOA) pose data file name
        public static String EOA_LOCALIZATION_DATA_FILE_NAME = "localizationData.csv";
        public static String EOA_LOCALIZATION_DATA_DELIMITER = ",";

    }

    //OTHER CONSTANTS

    public static double JOYSTICK_MINIMUM = 0.02;

    /// time driver has to enter the motif code manually
    /// <p>
    /// driver clicks buttons to enter the code
    public static int MOTIF_SELECTION_KEYBIND_TIME = 2250;

    public static double CONTROL_HUB_HZ = 80;

    public static int TELEMETRY_MS_TRANSMISSION_INTERVAL = 25;

    /// Height of goal in inches
    public static double HEIGHT_OF_GOAL = 53.996063;

    public static float TRIGGER_THRESHOLD = 0.2f;

    /// Index 0 is the left crservo.
    /// <p>
    /// Index 1 is the right crservo.
    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE
    };

    /// Index 0 is the left servo.
    /// <p>
    /// Index 1 is the right servo.
    public static Servo.Direction[] HOOD_ANGLER_SERVO_DIRECTIONS = {
            /*left motor*/ Servo.Direction.FORWARD,
            /*right motor*/ Servo.Direction.FORWARD
    };

    public static Servo.Direction LIFT_PTO_SERVO_DIRECTION = Servo.Direction.REVERSE;

    //public static double HOOD_ANGLER_INITIAL_RESETTING_POSITION = 0;

    //intake and transfer
    public static double INTAKE_POWER = 1;

    public static double REVERSE_INTAKE_POWER = -1;

    /// in milliseconds
    public static double IS_BALL_IN_INTAKE_DEADBAND_TIMER = 1200;

    public static double TRANSFER_VELOCITY = 2000;
    public static double REVERSE_TRANSFER_VELOCITY = -1600;
    public static double ANTI_TRANSFER_VELOCITY = -100;

    /// in milliseconds
    public static double FULLY_TRANSFER_TIME_SAFE = 1500;

    public static double[] TRANSFER_VELO_PIDF_COEFFICIENTS = {20, 7, 1, 5};

    @DeprecatedSystem(notes = "Not using PID in intake mode anymore as it's unnecessary.")
    public static double[] INTAKE_PIDF_DEFAULT_COEFFICIENTS = {20, 2.5, 0, 10};
    @DeprecatedSystem(notes = "Not using PID in intake mode anymore as it's unnecessary.")
    public static double[] INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER = {20, 0, 0, 10}; //integral is not being utilized

    //flywheel
    /// Index 0 is the left motor.
    /// <p>
    /// Index 1 is the right motor.
    public static DcMotorSimple.Direction[] FLYWHEEL_MOTOR_DIRECTIONS = {
            /*left motor*/ DcMotorSimple.Direction.REVERSE,
            /*right motor*/ DcMotorSimple.Direction.FORWARD
    };

    public static double[] FLYWHEEL_PIDVS_COEFFICIENTS = {
            0.00000465, 0.00000004, 0.0000215, 0.0000004, 0.00000141, 0.0001, 0.76, 0.833, 10_000
    };

    public static double[] FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS = {1800, 400, 45, 5 /*3.65*/};

    public static int FLYWHEEL_PIDFVAS_LOOP_TIME = 25;

    public static double FLYWHEEL_MIN_INTEGRAL_LIMIT = -0.5;
    public static double FLYWHEEL_MAX_INTEGRAL_LIMIT = 1;

    public static double FLYWHEEL_MIN_PROPORTIONAL_LIMIT = -0.1;
    public static double FLYWHEEL_MAX_PROPORTIONAL_LIMIT = 1;

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 1000;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 1000;

    public static double FLYWHEEL_VOLTAGE_FILTER_ALPHA = 0.13;

    //turret
    public static TurretBasePIDFSCoefficients TURRET_PIDFS_COEFFICIENTS = new TurretBasePIDFSCoefficients(
            0.0,
            new double[] {0.00000002, 0.00000002},
            new double[] {0.00000051, 0.00000043},
            0.0,
            null,
            0.04,
            new double[] {1000, 1000},
            new double[] {0.75, 0.75},
            new double[] {200, 200},
            new double[] {0.9, 0.9},
            1,
            new double[] {6, 3},
            1000,
            -0.3,
            0.3
    );

    //lift
    public static double[] LIFT_PIDFS_COEFFICIENTS = {0.1, 0.0275, 0.0013, 0.001, 1.67, 0.1, 0.85, 0.9, 195.0};

    public static double LIFT_MIN_INTEGRAL_LIMIT = -8;
    public static double LIFT_MAX_INTEGRAL_LIMIT = 8;

    /// This method is only to be used for when the lift hits the ground.
    /// @return kf from exponential regression.
    public static double getLiftKfFromRegression(double targetPosition) {
        return 0.11 * FastMath.pow(1.05, targetPosition);
    }

    public static double LIFT_POSITION = 550;

    public static double LIFT_PTO_ENGAGE_POSITION = 0.48;
    public static double LIFT_PTO_DISENGAGE_POSITION = 0.55;

    public static Map<String, EOAOffset> EOA_OFFSETS = new HashMap<>(
            Map.of("auto12", new EOAOffset(17.288, -32.73))
    );

}