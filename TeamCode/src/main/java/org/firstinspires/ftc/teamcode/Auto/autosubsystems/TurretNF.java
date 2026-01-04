package org.firstinspires.ftc.teamcode.Auto.autosubsystems;

import com.chaigptrobotics.shenanigans.No_u;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@No_u
public class TurretNF implements Subsystem {

    //Doesn't allow objects to be created
    private TurretNF() {}

    public static final TurretNF INSTANCE = new TurretNF();

    public TurretBase turret;

    @Override
    public void initialize() {

        turret = new TurretBase(ActiveOpMode.hardwareMap());

        turret.setPIDFSCoefficients(
                Constants.TURRET_PIDFS_COEFFICIENTS[0],
                Constants.TURRET_PIDFS_COEFFICIENTS[1],
                Constants.TURRET_PIDFS_COEFFICIENTS[2],
                Constants.TURRET_PIDFS_COEFFICIENTS[3],
                Constants.TURRET_PIDFS_COEFFICIENTS[4],
                Constants.TURRET_PIDFS_COEFFICIENTS[5],
                Constants.TURRET_PIDFS_COEFFICIENTS[6],
                Constants.TURRET_PIDFS_COEFFICIENTS[7],
                Constants.TURRET_PIDFS_COEFFICIENTS[8],
                Constants.TURRET_PIDFS_COEFFICIENTS[9],
                Constants.TURRET_PIDFS_COEFFICIENTS[10],
                Constants.TURRET_PIDFS_COEFFICIENTS[11],
                Constants.TURRET_PIDFS_COEFFICIENTS[12]
        );
        turret.setIConstraints(Constants.TURRET_MIN_INTEGRAL_LIMIT, Constants.TURRET_MAX_INTEGRAL_LIMIT);
        turret.reverse();

        goToHomePosition();
    }

    public Command setPosition(double position) {
        return new InstantCommand(() -> turret.setPosition(position - turret.startPosition));
    }

    ///  Tells turret to go to the start position
    public void goToHomePosition() {
        turret.setPosition(turret.startPosition);
    }

    @Override
    public void periodic() {

        turret.update();
    }
}
