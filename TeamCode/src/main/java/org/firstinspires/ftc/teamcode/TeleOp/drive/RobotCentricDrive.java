package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class RobotCentricDrive extends Subsystem {

    private DcMotor left_front, right_front, left_back, right_back;

    private BetterGamepad controller1;

    public void provideComponents(DcMotor left_front, DcMotor right_front, DcMotor left_back, DcMotor right_back, BetterGamepad controller1) {

        this.left_front = left_front;
        this.right_front = right_front;
        this.left_back = left_back;
        this.right_back = right_back;

        this.controller1 = controller1;
    }

    @Override
    public void update() {

        double lfPower = -controller1.left_stick_y() + controller1.right_stick_x() + controller1.left_stick_x();
        double lbPower = -controller1.left_stick_y() + controller1.right_stick_x() - controller1.left_stick_x();
        double rfPower = -controller1.left_stick_y() - controller1.right_stick_x() - controller1.left_stick_x();
        double rbPower = -controller1.left_stick_y() - controller1.right_stick_x() + controller1.left_stick_x();

        left_front.setPower(Math.abs(lfPower) > Constants.JOYSTICK_MINIMUM ? lfPower : 0);
        right_front.setPower(Math.abs(rfPower) > Constants.JOYSTICK_MINIMUM ? rfPower : 0);
        left_back.setPower(Math.abs(lbPower) > Constants.JOYSTICK_MINIMUM ? lbPower : 0);
        right_back.setPower(Math.abs(rbPower) > Constants.JOYSTICK_MINIMUM ? rbPower : 0);
    }
}
