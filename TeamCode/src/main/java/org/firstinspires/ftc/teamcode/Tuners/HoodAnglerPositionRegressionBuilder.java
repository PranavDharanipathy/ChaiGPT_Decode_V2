package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.util.MathUtil;

@Peak
@Config
public class HoodAnglerPositionRegressionBuilder {

    public static double HOOD_ANGLER_POSITION = 0.5;

    private HoodAngler hoodAngler;

    public void init(HardwareMap hardwareMap) {

        hoodAngler = new HoodAngler(hardwareMap, Constants.MapSetterConstants.hoodAnglerLeftServoDeviceName, Constants.MapSetterConstants.hoodAnglerRightServoDeviceName);
        hoodAngler.setServoDirections(Constants.HOOD_ANGLER_SERVO_DIRECTIONS);
    }

    public void runInstance(BetterGamepad controller) {

        if (controller.dpad_leftHasJustBeenPressed) {
            HOOD_ANGLER_POSITION+=0.05;
        }
        else if (controller.dpad_rightHasJustBeenPressed) {
            HOOD_ANGLER_POSITION-=0.05;
        }

        HOOD_ANGLER_POSITION = MathUtil.clamp(HOOD_ANGLER_POSITION, ShooterInformation.ShooterConstants.HOOD_ANGLER_MAX_POSITION, ShooterInformation.ShooterConstants.HOOD_ANGLER_MIN_POSITION);

        hoodAngler.setPosition(HOOD_ANGLER_POSITION);
    }

    public double $getPosition() {
        return hoodAngler.getPosition();
    }
}
