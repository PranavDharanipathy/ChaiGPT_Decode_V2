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

}
