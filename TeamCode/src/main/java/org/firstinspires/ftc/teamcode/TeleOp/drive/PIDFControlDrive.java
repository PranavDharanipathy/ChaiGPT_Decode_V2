package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class PIDFControlDrive extends Subsystem {

    private BetterGamepad controller1;

    private BasicVeloMotor left_front, left_back, right_front, right_back;

    public void provideComponents(BasicVeloMotor left_front, BasicVeloMotor right_front, BasicVeloMotor left_back, BasicVeloMotor right_back, BetterGamepad controller1) {

        this.left_front = left_front;
        this.right_front = right_front;
        this.left_back = left_back;
        this.right_back = right_back;

        this.left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        this.left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        this.controller1 = controller1;
    }

    @Override
    public void update() {

        double lfPower = controller1.left_stick_y() - controller1.right_stick_x() - controller1.left_stick_x();
        double lbPower = controller1.left_stick_y() - controller1.right_stick_x() + controller1.left_stick_x();
        double rfPower = controller1.left_stick_y() + controller1.right_stick_x() + controller1.left_stick_x();
        double rbPower = controller1.left_stick_y() + controller1.right_stick_x() - controller1.left_stick_x();

        left_front.setVelocity(Math.abs(lfPower) > Constants.DriveConstants.JOYSTICK_MINIMUM ? lfPower : 0);
        left_back.setVelocity(Math.abs(lbPower) > Constants.DriveConstants.JOYSTICK_MINIMUM ? lbPower : 0);
        right_front.setVelocity(Math.abs(rfPower) > Constants.DriveConstants.JOYSTICK_MINIMUM ? rfPower : 0);
        right_back.setVelocity(Math.abs(rbPower) > Constants.DriveConstants.JOYSTICK_MINIMUM ? rbPower : 0);
    }

}
