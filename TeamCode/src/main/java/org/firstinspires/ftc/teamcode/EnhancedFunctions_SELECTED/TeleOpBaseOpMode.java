package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;

import java.util.List;

public abstract class TeleOpBaseOpMode extends LinearOpMode {

    public TeleOpBaseOpMode() {}

    public volatile BetterGamepad controller1;
    public volatile BetterGamepad controller2;

    public volatile DcMotor left_front, right_front, left_back, right_back;

    public volatile BasicVeloMotor intake;
    public volatile BasicVeloMotor transfer;
    public volatile AdafruitBeambreakSensor intakeBeambreak, transferBeambreak;

    public volatile Limelight3A unstartedLimelight;

    public volatile TurretBase turret;

    public volatile HoodAngler hoodAngler;

    public volatile ExtremePrecisionFlywheel flywheel;

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

        left_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftFrontMotorDeviceName);
        right_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightFrontMotorDeviceName);
        left_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftBackMotorDeviceName);
        right_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightBackMotorDeviceName);

        intake = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.intakeMotorDeviceName);
        transfer = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.transferMotorDeviceName);

        unstartedLimelight = hardwareMap.get(Limelight3A.class, Constants.MapSetterConstants.limelight3AUSBDeviceName);

        intakeBeambreak = new AdafruitBeambreakSensor(hardwareMap, Constants.MapSetterConstants.intakeBeambreakSensorNames[0], Constants.MapSetterConstants.intakeBeambreakSensorNames[1]);
        transferBeambreak = new AdafruitBeambreakSensor(hardwareMap, Constants.MapSetterConstants.transferBeambreakSensorNames[0], Constants.MapSetterConstants.transferBeambreakSensorNames[1]);

        flywheel = new ExtremePrecisionFlywheel(
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        turret = new TurretBase(hardwareMap, Constants.MapSetterConstants.turretBaseLeftServoDeviceName, Constants.MapSetterConstants.turretBaseRightServoDeviceName);

        hoodAngler = new HoodAngler(hardwareMap, Constants.MapSetterConstants.hoodAnglerLeftServoDeviceName, Constants.MapSetterConstants.hoodAnglerRightServoDeviceName);

        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);
    }

    /// Provide traits
    public void applyComponentTraits() {

        left_back.setDirection(DcMotor.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        intake.setVelocityPIDFCoefficients(
                Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[0],
                Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[1],
                Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[2],
                Constants.INTAKE_PIDF_DEFAULT_COEFFICIENTS[3]
        );

        transfer.setVelocityPIDFCoefficients(
                Constants.TRANSFER_VELO_PIDF_COEFFICIENTS[0],
                Constants.TRANSFER_VELO_PIDF_COEFFICIENTS[1],
                Constants.TRANSFER_VELO_PIDF_COEFFICIENTS[2],
                Constants.TRANSFER_VELO_PIDF_COEFFICIENTS[3]
        );

        unstartedLimelight.setPollRateHz(ShooterInformation.CameraConstants.CAMERA_POLL_RATE);

        flywheel.setInternalParameters(
                ShooterInformation.ShooterConstants.getTotalFlywheelAssemblyWeight(),
                ShooterInformation.ShooterConstants.SHAFT_DIAMETER,
                ShooterInformation.ShooterConstants.MOTOR_CORE_VOLTAGE,
                ShooterInformation.ShooterConstants.MOTOR_RPM,
                ShooterInformation.ShooterConstants.BURST_DECELERATION_RATE
                );
        flywheel.setVelocityPIDFVASCoefficients(
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[0],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[1],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[2],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[3],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[4],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[5],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[6],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[7],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[8]
        );
        flywheel.setIConstraints(Constants.FLYWHEEL_MIN_INTEGRAL_LIMIT, Constants.FLYWHEEL_MAX_INTEGRAL_LIMIT);

        turret.setPIDFCoefficients(
                Constants.TURRET_PIDF_COEFFICIENTS[0],
                Constants.TURRET_PIDF_COEFFICIENTS[1],
                Constants.TURRET_PIDF_COEFFICIENTS[2],
                Constants.TURRET_PIDF_COEFFICIENTS[3]
        );
        turret.setIConstraints(Constants.TURRET_MIN_INTEGRAL_LIMIT, Constants.TURRET_MAX_INTEGRAL_LIMIT);

        hoodAngler.setServoDirections(Constants.HOOD_ANGLER_SERVO_DIRECTIONS);
    }

}