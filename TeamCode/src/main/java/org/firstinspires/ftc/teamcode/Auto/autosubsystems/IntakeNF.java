package org.firstinspires.ftc.teamcode.Auto.autosubsystems;

import com.chaigptrobotics.shenanigans.No_u;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.LiftPTO;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@No_u
public class IntakeNF implements Subsystem {

    //Doesn't allow objects to be created
    private IntakeNF() {}

    public static final IntakeNF INSTANCE = new IntakeNF();

    public MotorEx intake;
    private LiftPTO liftPTO; //intentionally private

    @Override
    public void initialize() {

        intake = new MotorEx(Constants.MapSetterConstants.intakeMotorDeviceName);
        intake.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        liftPTO = new LiftPTO(ActiveOpMode.hardwareMap());
        liftPTO.setState(LiftPTO.PTOState.DISENGAGE);
    }

    public Command customPower(double power) {
        return new SetPower(intake, power);
    }

    public Command intake() {
        return new SetPower(intake, Constants.INTAKE_POWER);
    }

    public Command reverse() {
        return new SetPower(intake, Constants.REVERSE_INTAKE_POWER);
    }

    public void end() {
        intake.setPower(0);
    }

    @Override
    public void periodic() {

    }
}
