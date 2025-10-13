package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.TeleOp.drive.HolonomicDrive;

@Config
public class Constants {

    public static class DriveConstants {

        public enum HolonomicDriveSidesReversed {

            FRONT_MOTOR(MapSetterConstants.leftFrontMotorDeviceName), BACK_MOTOR(MapSetterConstants.leftBackMotorDeviceName);

            private String motorName;

            HolonomicDriveSidesReversed(String motorName) {
                this.motorName = motorName;
            }

            public String getStringValue() {
                return motorName;
            }
        }

        public static HolonomicDrive.TurnPolarity leftFrontTurnPolarity = HolonomicDrive.TurnPolarity.POSITIVE;
        public static HolonomicDrive.TurnPolarity rightFrontTurnPolarity = HolonomicDrive.TurnPolarity.NEGATIVE;
        public static HolonomicDrive.TurnPolarity leftBackTurnPolarity = HolonomicDrive.TurnPolarity.POSITIVE;
        public static HolonomicDrive.TurnPolarity rightBackTurnPolarity = HolonomicDrive.TurnPolarity.NEGATIVE;
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
    }



    //OTHER CONSTANTS

    public static int CONTROL_HUB_HZ = 80;

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

}
