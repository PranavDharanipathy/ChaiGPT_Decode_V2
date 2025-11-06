package org.firstinspires.ftc.teamcode.TeleOp.drive;

import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.Subsystem;

public class FieldCentricDrive extends Subsystem {

    private DcMotor left_front, right_front, left_back, right_back;

    private Rev9AxisImu rev9AxisImu;

    private BetterGamepad controller1;

    public void provideComponents(DcMotor left_front, DcMotor right_front, DcMotor left_back, DcMotor right_back, Rev9AxisImu rev9AxisImu, BetterGamepad controller1) {

        this.left_front = left_front;
        this.right_front = right_front;
        this.left_back = left_back;
        this.right_back = right_back;

        this.rev9AxisImu = rev9AxisImu;

        this.controller1 = controller1;
    }

    private double startHeading = 0;

    @Override
    public void update() {

        double yaw = rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        if (controller1.aHasJustBeenPressed) startHeading = yaw;
//
//        double yawDifference = startHeading - yaw;
//
//        double forwardJoystickAmount = Math.abs(controller1.left_stick_y()) > Constants.JOYSTICK_MINIMUM ? -controller1.left_stick_y() : 0;
//        double strafeJoystickAmount = Math.abs(controller1.left_stick_x()) > Constants.JOYSTICK_MINIMUM ? controller1.left_stick_x() : 0;
//        double turnJoystickAmount = Math.abs(controller1.right_stick_x()) > Constants.JOYSTICK_MINIMUM ? controller1.right_stick_x() : 0;
//
//        double directionalPower = Math.hypot(forwardJoystickAmount, strafeJoystickAmount);
//
//        double forward = directionalPower * Math.sin(yawDifference);
//        double strafe = directionalPower * Math.cos(yawDifference);
//
//        double lfPower = forward + strafe + turnJoystickAmount;
//        double lbPower = forward + strafe - turnJoystickAmount;
//        double rfPower = forward - strafe - turnJoystickAmount;
//        double rbPower = forward - strafe + turnJoystickAmount;
//
//        left_front.setPower(Math.abs(lfPower));
//        right_front.setPower(Math.abs(rfPower));
//        left_back.setPower(Math.abs(lbPower));
//        right_back.setPower(Math.abs(rbPower));
//----------------------------------------------------------------------------------
        double y = -controller1.left_stick_y(); // Remember, Y stick value is reversed
        double x = controller1.left_stick_x();
        double rx = controller1.right_stick_x();

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (controller1.aHasJustBeenPressed) startHeading = yaw;

        double botHeading = rev9AxisImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(startHeading-botHeading) - y * Math.sin(startHeading-botHeading);
        double rotY = x * Math.sin(startHeading-botHeading) + y * Math.cos(startHeading-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        left_front.setPower(frontLeftPower);
        left_back.setPower(backLeftPower);
        right_front.setPower(frontRightPower);
        right_back.setPower(backRightPower);
    }
}
