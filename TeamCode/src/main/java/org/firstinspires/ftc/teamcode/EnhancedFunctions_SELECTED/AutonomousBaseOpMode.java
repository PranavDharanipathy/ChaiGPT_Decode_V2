package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.ForAutonomousRobotReset;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.RobotResetter;

import java.util.List;

public abstract class AutonomousBaseOpMode extends LinearOpMode {

    public AutonomousBaseOpMode() {}

    public volatile Telemetry telemetry;

    public volatile BetterGamepad controller1;
    public volatile BetterGamepad controller2;

    public volatile BasicVeloMotor intake;
    public volatile BasicVeloMotor transfer;
    public volatile AdafruitBeambreakSensor intakeBeambreak, transferBeambreak;

    public volatile TurretBase turret;

    public volatile HoodAngler hoodAngler;

    public volatile ExtremePrecisionFlywheel flywheel;

    public void fullInit() {

        initialize();
        applyComponentTraits();

        RobotResetter forAutonomousRobotReset = new ForAutonomousRobotReset(hoodAngler);
    }

    /// Initializing devices
    private void initialize() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.intakeMotorDeviceName);
        transfer = new BasicVeloMotor(hardwareMap, Constants.MapSetterConstants.transferMotorDeviceName);

        intakeBeambreak = new AdafruitBeambreakSensor(hardwareMap, Constants.MapSetterConstants.intakeBeambreakSensorNames[0], Constants.MapSetterConstants.intakeBeambreakSensorNames[1]);
        transferBeambreak = new AdafruitBeambreakSensor(hardwareMap, Constants.MapSetterConstants.transferBeambreakSensorNames[0], Constants.MapSetterConstants.transferBeambreakSensorNames[1]);

        flywheel = new ExtremePrecisionFlywheel(
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        turret = new TurretBase(hardwareMap);

        hoodAngler = new HoodAngler(hardwareMap, Constants.MapSetterConstants.hoodAnglerLeftServoDeviceName, Constants.MapSetterConstants.hoodAnglerRightServoDeviceName);

        controller1 = new BetterGamepad(gamepad1);
        controller2 = new BetterGamepad(gamepad2);
    }

    /// Provide traits
    private void applyComponentTraits() {

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
                Constants.TURRET_PIDF_COEFFICIENTS[3],
                Constants.TURRET_PIDF_COEFFICIENTS[4],
                Constants.TURRET_PIDF_COEFFICIENTS[5],
                Constants.TURRET_PIDF_COEFFICIENTS[6]
        );
        turret.setIConstraints(Constants.TURRET_MIN_INTEGRAL_LIMIT, Constants.TURRET_MAX_INTEGRAL_LIMIT);
        turret.reverse();

        hoodAngler.setServoDirections(Constants.HOOD_ANGLER_SERVO_DIRECTIONS);
    }

}