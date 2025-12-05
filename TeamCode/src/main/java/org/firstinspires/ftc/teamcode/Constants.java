package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeleOp.Obelisk;

@Config
public class Constants {

    public static class DriveConstants {


    }

    public static class IMUConstants {

        public static RevHubOrientationOnRobot CONTROL_HUB_ORIENTATION_IMU_PARAMETER =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        public static Rev9AxisImuOrientationOnRobot REV_9_AXIS_IMU_ORIENTATION_IMU_PARAMETER =
                new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.BACKWARD);

        public static Orientation getInternalIMUOrientationStats(IMU internalIMU) {

            return internalIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        }

        public static Orientation getRev9AxisIMUOrientationStats(Rev9AxisImu rev9AxisIMU) {

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
        public static String transferMotorDeviceName = "transfer";

        public static String[] transferBeambreakSensorNames = {"transfer_beambreak_power", "transfer_beambreak_receiver"};

        public static String[] intakeBeambreakSensorNames = {"intake_beambreak_power", "intake_beambreak_receiver"};

        public static String[] rev2mDistanceSensorNames = {"left_distance_sensor", "back_distance_sensor", "right_distance_sensor"};
    }


    //OTHER CONSTANTS

    public static double JOYSTICK_MINIMUM = 0.02;

    /// name of the obelisk xml file
    public static String OBELISK_XML_FILE_NAME = "obelisk";
    /// the key used when saving and loading data to the obelisk xml file
    public static String OBELISK_XML_DATA_KEY = "id";
    /// defaults to INVALID
    public static int OBELISK_XML_DEFAULT_KEY = Obelisk.OBELISK.INVALID.getAprilTagNumber();

    /// time driver has to enter the obelisk code manually
    /// <p>
    /// driver clicks multiple buttons to enter the code
    public static int OBELISK_SELECTION_KEYBIND_TIME = 2250;

    public static double CONTROL_HUB_HZ = 80;

    public enum HUB_TYPE {

        CONTROL_HUB("Control Hub"), EXPANSION_HUB("Expansion Hub");

        private String hubName;

        HUB_TYPE(String hubName) {
            this.hubName = hubName;
        }

        public String getGivenName() {
            return hubName;
        }
    }

    /// Height of goal in inches
    public static double HEIGHT_OF_GOAL = 53.996063;

    /// Height of AprilTag on goal in inches
    public static double HEIGHT_OF_GOAL_APRIL_TAG = 27;

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

    public static double HOOD_ANGLER_INITIAL_RESETTING_POSITION = 0;

    //intake and transfer
    public static double BASE_INTAKE_VELOCITY = 2000;

    public static double INTAKE_VELOCITY_WHEN_BALL_IN_TRANSFER = 1200;

    public static double REVERSE_INTAKE_VELOCITY = -1200;

    /// in milliseconds
    public static double IS_BALL_IN_INTAKE_DEADBAND_TIMER = 1200;

    public static double TRANSFER_VELOCITY = 1200;
    public static double REVERSE_TRANSFER_VELOCITY = -1600;
    public static double ANTI_TRANSFER_VELOCITY = -250;

    // DIFFERENT TUNED VALUEs
    public static double[] MECANUM_DRIVE_NORMAL_FEEDFORWARD  = {0.0003075, 0.9, 0.00008};

    public static double[] MECANUM_DRIVE_FAST_FEEDFORWARD = {};

    public static double[] MECANUM_DRIVE_NORMAL_GAINS = {};

    public static double[] MECANUM_DRIVE_FAST_GAINS = {};

    public static double[] MECANUM_DRIVE_NORMAL_VEL_GAINS = {};

    public static double[] MECANUM_DRIVE_NORMAL_FAST_GAINS = {};

    /// in milliseconds
    public static double FULLY_TRANSFER_TIME = 2000;

    public static double[] TRANSFER_VELO_PIDF_COEFFICIENTS = {20, 7, 1, 5};

    public static double[] INTAKE_PIDF_DEFAULT_COEFFICIENTS = {20, 2.5, 0, 10};
    public static double[] INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER = {20, 0, 0, 10}; //integral is not being utilized

    //flywheel
    /// Index 0 is the left motor.
    /// <p>
    /// Index 1 is the right motor.
    public static DcMotorSimple.Direction[] FLYWHEEL_MOTOR_DIRECTIONS = {
            /*left motor*/ DcMotorSimple.Direction.REVERSE,
            /*right motor*/ DcMotorSimple.Direction.FORWARD
    };

    public static double[] FLYWHEEL_PIDFVAS_COEFFICIENTS = {
            0.000125, 0.0000005, 0, 0.00003, 0.0, 0.00000173, 0.0, 0.00013, 0.9, 0.1, 7200 //0.000001, 0.000001, 0.0000004, 0.0000001, 0.0, 0.00000173, 0.0, 0.00008, 0.9, 0.1, 7200
    };

    public static double[] FLYWHEEL_VELOCITY_KALMAN_FILTER_PARAMETERS = {0.92, 20, 17, 2.5, 5.3455149501661126};

    public static int FLYWHEEL_PIDFVAS_LOOP_TIME = 25;

    public static double FLYWHEEL_MIN_INTEGRAL_LIMIT = -0.05;
    public static double FLYWHEEL_MAX_INTEGRAL_LIMIT = 1;

    public static double FLYWHEEL_MIN_PROPORTIONAL_LIMIT = -0.04;
    public static double FLYWHEEL_MAX_PROPORTIONAL_LIMIT = 1;

    public static double FLYWHEEL_BURST_DECELERATION_RATE = 250;

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 4400;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 4400;

    public static double FLYWHEEL_VOLTAGE_FILTER_ALPHA = 0.02;

    //turret
    public static Double[] TURRET_PIDFS_COEFFICIENTS = {0.000135, 0.0000001475, 0.00575, null, 0.014, 0.6, 0.85, 0.99, -2150.0};

    public static double TURRET_MIN_INTEGRAL_LIMIT = -1;
    public static double TURRET_MAX_INTEGRAL_LIMIT = 1;

}
