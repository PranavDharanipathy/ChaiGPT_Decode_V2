package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;

import java.util.List;

public abstract class TeleOpBaseOpMode extends LinearOpMode {

    public TeleOpBaseOpMode() {}

    public volatile BetterGamepad controller1;
    public volatile BetterGamepad controller2;

    public volatile BasicVeloMotor intake, transfer;

    public volatile AdafruitBeambreakSensor intakeBeambreak, transferBeambreak;

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

        intake = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.intakeMotorDeviceName);
        transfer = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.transferMotorDeviceName);

        intakeBeambreak = new AdafruitBeambreakSensor(hardwareMap, Constants.MapSetterConstants.intakeBeambreakSensorNames[0], Constants.MapSetterConstants.intakeBeambreakSensorNames[1]);
        transferBeambreak = new AdafruitBeambreakSensor(hardwareMap, Constants.MapSetterConstants.transferBeambreakSensorNames[0], Constants.MapSetterConstants.transferBeambreakSensorNames[1]);

        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);
    }

    /// Provide traits
    public void applyComponentTraits() {

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setVelocityPIDFCoefficients(
                Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[0],
                Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[1],
                Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[2],
                Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[3]
        );

        transfer.setVelocityPIDFCoefficients(
                Constants.TRANSFER_PIDF_COEFFICIENTS[0],
                Constants.TRANSFER_PIDF_COEFFICIENTS[1],
                Constants.TRANSFER_PIDF_COEFFICIENTS[2],
                Constants.TRANSFER_PIDF_COEFFICIENTS[3]
        );
    }

}