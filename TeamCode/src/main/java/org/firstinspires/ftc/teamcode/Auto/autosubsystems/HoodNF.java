package org.firstinspires.ftc.teamcode.Auto.autosubsystems;

import com.chaigptrobotics.shenanigans.No_u;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@No_u
public class HoodNF implements Subsystem {

    //Doesn't allow objects to be created
    private HoodNF() {}

    public static final HoodNF INSTANCE = new HoodNF();

    public HoodAngler hood;

    @Override
    public void initialize() {
        hood = new HoodAngler(ActiveOpMode.hardwareMap(), Constants.MapSetterConstants.hoodAnglerLeftServoDeviceName, Constants.MapSetterConstants.hoodAnglerRightServoDeviceName);
        hood.setServoDirections(Constants.HOOD_ANGLER_SERVO_DIRECTIONS);
        hood.setPosition(0.4); //init position
    }

    public Command setPosition(double position) {
        return new InstantCommand(() -> hood.setPosition(position));
    }

    @Override
    public void periodic() {

    }
}
