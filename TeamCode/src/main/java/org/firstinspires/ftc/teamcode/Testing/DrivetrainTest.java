package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;

@TeleOp(group = "testing")
public class DrivetrainTest extends OpMode {

    private DcMotor lf, rf, lb, rb;

    private BetterGamepad controller1;

    @Override
    public void init() {

        lf = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftFrontMotorDeviceName);
        rf = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightFrontMotorDeviceName);
        lb = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftBackMotorDeviceName);
        rb = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightBackMotorDeviceName);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller1 = new BetterGamepad(gamepad1);
    }

    @Override
    public void loop() {

        double lfPower = controller1.left_stick_y() - controller1.right_stick_x() - controller1.left_stick_x();
        double lbPower = controller1.left_stick_y() - controller1.right_stick_x() + controller1.left_stick_x();
        double rfPower = controller1.left_stick_y() + controller1.right_stick_x() + controller1.left_stick_x();
        double rbPower = controller1.left_stick_y() + controller1.right_stick_x() - controller1.left_stick_x();

        lf.setPower(Math.abs(lfPower) > Constants.DriveConstants.JOYSTICK_MINIMUM ? lfPower : 0);
        rf.setPower(Math.abs(rfPower) > Constants.DriveConstants.JOYSTICK_MINIMUM ? rfPower : 0);
        lb.setPower(Math.abs(lbPower) > Constants.DriveConstants.JOYSTICK_MINIMUM ? lbPower : 0);
        rb.setPower(Math.abs(rbPower) > Constants.DriveConstants.JOYSTICK_MINIMUM ? rbPower : 0);
    }
}
