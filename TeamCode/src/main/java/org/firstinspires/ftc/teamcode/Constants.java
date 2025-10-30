package org.firstinspires.ftc.teamcode;

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

public class Constants {

    public static class DriveConstants {

        public static double[] LEFT_SIDE_PIDF = {0,0,0,0};
        public static double[] RIGHT_SIDE_PIDF = {0,0,0,0};
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
    /// defaults to INVALID which is -1
    public static int OBELISK_XML_DEFAULT_KEY = -1;

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

    public static float TRIGGER_THRESHOLD = 0.07f;

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

    public static double TRANSFER_VELOCITY = 1600;
    public static double REVERSE_TRANSFER_VELOCITY = -1600;
    public static double ANTI_TRANSFER_VELOCITY = -200;

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
            0.0002, 0.00064, 0.0000075, 0.015, 0.0000241, 0.000000075, 0.0001, 0.9, 0.175
    };

    public static int FLYWHEEL_PIDFVAS_LOOP_TIME = 40;

    public static double FLYWHEEL_MIN_INTEGRAL_LIMIT = -0.35;
    public static double FLYWHEEL_MAX_INTEGRAL_LIMIT = 0.35;

    public static double BURST_DECELERATION_RATE = 250;

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 4400;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 4400;

    //turret
    public static double[] TURRET_PIDF_COEFFICIENTS = {0.0003, 0, 0.00003, 0}; // the best results were with 0 integral so that what it's going to be I guess

    public static double TURRET_MIN_INTEGRAL_LIMIT = -1;
    public static double TURRET_MAX_INTEGRAL_LIMIT = 1;

    public static double TURRET_MANUAL_ADJUSTMENT = 1;

}
