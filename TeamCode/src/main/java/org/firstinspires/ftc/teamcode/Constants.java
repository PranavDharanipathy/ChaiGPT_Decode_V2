package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;

public class Constants {

    public static class DriveConstants {


    }

    public static class MapSetterConstants {


        public static LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

        public static String leftFrontMotorDeviceName = "left_front";
        public static String leftBackMotorDeviceName = "left_back";
        public static String rightFrontMotorDeviceName = "right_front";
        public static String rightBackMotorDeviceName = "right_back";

        public static String intakeMotorDeviceName = "";
    }



    //OTHER CONSTANTS

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

}
