package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class ProfiledFlywheel {

    private ExtremePrecisionFlywheel flywheel;

    public ExtremePrecisionFlywheel accessFlywheel() {
        return flywheel;
    }


    public ProfiledFlywheel(HardwareMap hardwareMap) {

        flywheel = new ExtremePrecisionFlywheel(
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName),
                hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setInternalParameters(
                ShooterInformation.ShooterConstants.getTotalFlywheelAssemblyWeight(),
                ShooterInformation.ShooterConstants.SHAFT_DIAMETER,
                ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_RPM
        );
        flywheel.setVelocityPIDVSCoefficients(
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[0],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[1],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[2],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[3],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[4],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[5],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[6],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[7],
                Constants.FLYWHEEL_PIDVS_COEFFICIENTS[8]
        );
        flywheel.setVoltageFilterAlpha(Constants.FLYWHEEL_VOLTAGE_FILTER_ALPHA);
        flywheel.setIConstraints(Constants.FLYWHEEL_MIN_INTEGRAL_LIMIT, Constants.FLYWHEEL_MAX_INTEGRAL_LIMIT);
        flywheel.setPConstraints(Constants.FLYWHEEL_MIN_PROPORTIONAL_LIMIT, Constants.FLYWHEEL_MAX_PROPORTIONAL_LIMIT);

    }

    public void update() {

        flywheel.updateKvBasedOnVoltage();
        flywheel.update();


    }
}
