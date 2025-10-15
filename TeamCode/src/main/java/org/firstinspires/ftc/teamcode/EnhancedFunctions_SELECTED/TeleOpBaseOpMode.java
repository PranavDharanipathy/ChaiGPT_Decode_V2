package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.List;

public abstract class TeleOpBaseOpMode extends LinearOpMode {

    public TeleOpBaseOpMode() {}

    public volatile BetterGamepad controller1;
    public volatile BetterGamepad controller2;

    private volatile List<LynxModule> robotHubs;

    /// initializes/creates LynxModule
    public void setUpLynxModule() {

        robotHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : robotHubs) {
            hub.setBulkCachingMode(Constants.MapSetterConstants.bulkCachingMode);
        }
    }

    /// Clears cache of LynxModule
    public void clearCacheOfLynxModule() {

        for (LynxModule hub : robotHubs) {
            hub.clearBulkCache();
        }
    }

    /// Closes LynxModule
    public void closeLynxModule() {

        for (LynxModule hub : robotHubs) {
            hub.close();
        }
    }

    /// Initializing devices
    public void initializeDevices() {



        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);
    }

    /// Provide traits
    public void applyComponentTraits() {


    }

}