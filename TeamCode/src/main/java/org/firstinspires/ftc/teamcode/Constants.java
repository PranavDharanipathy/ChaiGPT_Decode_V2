package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class Constants {

    public static class DriveConstants {

        public static double JOYSTICK_MINIMUM = 0.02;

        public static double[] LEFT_SIDE_PIDF = {0,0,0,0};
        public static double[] RIGHT_SIDE_PIDF = {0,0,0,0};

    }

    public static class MapSetterConstants {

        public static LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

        public static String leftFrontMotorDeviceName = "left_front";
        public static String leftBackMotorDeviceName = "left_back";
        public static String rightFrontMotorDeviceName = "right_front";
        public static String rightBackMotorDeviceName = "right_back";

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
    }



    //OTHER CONSTANTS

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

    /// Index 0 is the left motor.
    /// <p>
    /// Index 1 is the right motor.
    public static DcMotorSimple.Direction[] FLYWHEEL_MOTOR_DIRECTIONS = {
            /*left motor*/ DcMotorSimple.Direction.REVERSE,
            /*right motor*/ DcMotorSimple.Direction.FORWARD
    };

    public static DcMotorSimple.Direction TURRET_BASE_DIRECTION = DcMotorSimple.Direction.FORWARD;


    public static float TRIGGER_THRESHOLD = 0.07f;

    public static double BASE_INTAKE_VELOCITY = 2000;

    public static double INTAKE_VELOCITY_WHEN_BALL_IN_TRANSFER = 1200;

    public static double REVERSE_INTAKE_VELOCITY = -1200;

    /// in milliseconds
    public static long IS_BALL_IN_INTAKE_DEADBAND_TIMER = 400;
    public static double TRANSFER_VELOCITY = 1200;

    // public static double SHOOTER_THRESHOLD = 1f;

    public static double[] TRANSFER_VELO_PIDF_COEFFICIENTS = {20, 7, 1, 5};
    public static double[] TRANSFER_POSITIONAL_PIDF_COEFFICIENTS = {};

    public static double[] INTAKE_PIDF_DEFAULT_COEFFICIENTS = {20, 2.5, 0, 10};
    public static double[] INTAKE_PIDF_COEFFICIENTS_WHEN_BALL_IS_IN_TRANSFER = {20, 0, 0, 10}; //integral is not being utilized



}
