package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Auto.autosubsystems.FlywheelNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.HoodNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.IntakeNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.TransferNF;
import org.firstinspires.ftc.teamcode.Auto.autosubsystems.TurretNF;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

/// He da big R, big R for Robot
public class RobotNF extends SubsystemGroup {


    private RobotNF() {

        super(
                IntakeNF.INSTANCE,
                TransferNF.INSTANCE,
                HoodNF.INSTANCE,
                TurretNF.INSTANCE,
                FlywheelNF.INSTANCE
        );
    }

    public static final RobotNF robot = new RobotNF();

    //intake
    public final Command intake() {
        return new InstantCommand(IntakeNF.INSTANCE::intake);
    }

    public final Command reverseIntake() {
        return new InstantCommand(IntakeNF.INSTANCE::reverse);
    }

    public final Command intake(double power) {
        return new InstantCommand(() -> IntakeNF.INSTANCE.customPower(power));
    }

    //turret
    public final Command turretTo(double position) {
        return new InstantCommand(() -> TurretNF.INSTANCE.setPosition(position));
    }

    //flywheel
    public final Command setFlywheelVel(double vel) {
        return new InstantCommand(() -> FlywheelNF.INSTANCE.setVel(vel, false));
    }

    /// AIR means allow integral reset
    public final Command setFlywheelVelAIR(double vel) {
        return new InstantCommand(() -> FlywheelNF.INSTANCE.setVel(vel, true));
    }

    public final Command waitTilFlywheelAtVel() {
        return new WaitUntil(() -> FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= FlywheelNF.INSTANCE.flywheel.getTargetVelocity());
    }

    public final Command waitTilFlywheelAtVel(double vel) {
        return new WaitUntil(() -> FlywheelNF.INSTANCE.flywheel.getRealVelocity() >= vel);
    }

    //transfer
    public final Command transfer(double transferTime, double timeBetweenTransfers) {

        return new SequentialGroup(
                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.anti(),

                new Delay(timeBetweenTransfers),

                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.anti(),

                new Delay(timeBetweenTransfers),

                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.anti()
        );
    }

    public final Command transfer(double transferTime, double timeBetweenTransfers, double distance, PathChain pathChain) {

        return new SequentialGroup(
                new WaitUntil(() -> pathChain.lastPath().getDistanceRemaining() <= distance),
                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.anti(),

                new Delay(timeBetweenTransfers),

                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.anti(),

                new Delay(timeBetweenTransfers),

                TransferNF.INSTANCE.transfer(),
                new Delay(transferTime),
                TransferNF.INSTANCE.anti()
        );
    }

    //hood
    public final Command hoodTo(double position) {
        return new InstantCommand(() -> HoodNF.INSTANCE.setPosition(position));
    }


    public final Command stop() {

        return new ParallelGroup(
                IntakeNF.INSTANCE.stop(),
                TransferNF.INSTANCE.stop(),
                TurretNF.INSTANCE.goToHomePosition(),
                FlywheelNF.INSTANCE.stop()
        );
    }
}
