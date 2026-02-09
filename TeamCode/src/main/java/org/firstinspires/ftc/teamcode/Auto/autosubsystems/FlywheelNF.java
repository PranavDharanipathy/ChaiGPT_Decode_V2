package org.firstinspires.ftc.teamcode.Auto.autosubsystems;

import com.chaigptrobotics.shenanigans.No_u;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.ShooterInformation;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@No_u
public class FlywheelNF implements Subsystem {

    //Doesn't allow objects to be created
    private FlywheelNF() {}

    public static final FlywheelNF INSTANCE = new FlywheelNF();

    public ExtremePrecisionFlywheel flywheel;

    @Override
    public void initialize() {

        flywheel = new ExtremePrecisionFlywheel(
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, Constants.MapSetterConstants.leftFlywheelMotorDeviceName),
                ActiveOpMode.hardwareMap().get(DcMotorEx.class, Constants.MapSetterConstants.rightFlywheelMotorDeviceName)
        );

        flywheel.setInternalParameters(
                ShooterInformation.ShooterConstants.getTotalFlywheelAssemblyWeight(),
                ShooterInformation.ShooterConstants.SHAFT_DIAMETER,
                ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_CORE_VOLTAGE,
                ShooterInformation.ShooterConstants.FLYWHEEL_MOTOR_RPM,
                ShooterInformation.ShooterConstants.BURST_DECELERATION_RATE
        );
        flywheel.initVoltageSensor(ActiveOpMode.hardwareMap());
        flywheel.setVoltageFilterAlpha(Constants.FLYWHEEL_VOLTAGE_FILTER_ALPHA);
        flywheel.setVelocityPIDFVASCoefficients(
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[0],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[1],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[2],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[3],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[4],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[5],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[6],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[7],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[8],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[9],
                Constants.FLYWHEEL_PIDFVAS_COEFFICIENTS[10]
        );

        flywheel.setIConstraints(Constants.FLYWHEEL_MIN_INTEGRAL_LIMIT, Constants.FLYWHEEL_MAX_INTEGRAL_LIMIT);
        flywheel.setPConstraints(Constants.FLYWHEEL_MIN_PROPORTIONAL_LIMIT, Constants.FLYWHEEL_MAX_PROPORTIONAL_LIMIT);

        flywheel.reset();
    }
    public Command setVel(double vel, boolean allowIntegralReset) {
        return new InstantCommand(() -> flywheel.setVelocity(vel, allowIntegralReset));
    }

    private double velWanted, velInflated, velSwitch;
    private boolean useVelCatching = false;

    public void setVelCatch(double m_velWanted, double m_velInflated, double m_velSwitch) {

        useVelCatching = true;
        velWanted = m_velWanted;
        velInflated = m_velInflated;
        velSwitch = m_velSwitch;
    }

    public Command setVelCatchCmd(double m_velWanted, double m_velInflated, double m_velSwitch) {

        return new Command() {

            @Override
            public boolean isDone() {

                useVelCatching = true;
                velWanted = m_velWanted;
                velInflated = m_velInflated;
                velSwitch = m_velSwitch;

                return true;
            }
        };
    }

    public void end() {
        flywheel.setVelocity(0, true);
    }

    @Override
    public void periodic() {

        if (useVelCatching) {

            if (Math.abs(flywheel.getTargetVelocity() - flywheel.getRealVelocity()) > velSwitch) {
                flywheel.setVelocity(velInflated, false);
            }
            else {
                flywheel.setVelocity(velWanted, false);
            }
        }

        flywheel.updateKvBasedOnVoltage();
        flywheel.update();
    }
}
