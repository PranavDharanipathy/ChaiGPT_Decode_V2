package org.firstinspires.ftc.teamcode.TurretSystems;

import org.firstinspires.ftc.teamcode.TurretSystems.ShooterInformation.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.SimpleMathUtil;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Peak
/// Uses AXONs
public class HoodAngler {

    private ServoImplEx leftHoodAngler;
    private ServoImplEx rightHoodAngler;

    public HoodAngler(HardwareMap hardwareMap, String leftHoodAnglerServoName, String rightHoodAnglerServoName) {

        leftHoodAngler = hardwareMap.get(ServoImplEx.class, leftHoodAnglerServoName);
        rightHoodAngler = hardwareMap.get(ServoImplEx.class, rightHoodAnglerServoName);

        leftHoodAngler.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));
        rightHoodAngler.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));
    }

    /// The first item is for the left angler servo and the seconds item is for the right angler servo
    public void setServoDirections(Servo.Direction[] directions) {

        leftHoodAngler.setDirection(directions[0]); //1st item
        rightHoodAngler.setDirection(directions[1]); //2nd item
    }

    public Servo.Direction[] getServoDirections() {

        return new Servo.Direction[] {
                leftHoodAngler.getDirection(), //1st item
                rightHoodAngler.getDirection() //2nd item
        };
    }

    private double position;

    public void $setPosition(double position) {

        leftHoodAngler.setPosition(position);
        rightHoodAngler.setPosition(position);
    }

    /// @param angle in degrees
    public void setAngle(double angle) {

        //get raw value
        double rawPosition = (angle + ShooterConstants.TURRET_DEGREES_PER_HOOD_ANGLER_DEGREE) * ShooterConstants.HOOD_ANGLER_POSITIONAL_INCREMENT_PER_DEGREE;

        //clamp value
        position = SimpleMathUtil.clamp(rawPosition, ShooterConstants.HOOD_ANGLER_MIN_POSITION_LIMIT, ShooterConstants.HOOD_ANGLER_MAX_POSITION_LIMIT);

        leftHoodAngler.setPosition(position);
        rightHoodAngler.setPosition(position);
    }

    /// @return in degrees
    public double getAngle() {
        return (position / ShooterConstants.HOOD_ANGLER_POSITIONAL_INCREMENT_PER_DEGREE) - ShooterConstants.TURRET_DEGREES_PER_HOOD_ANGLER_DEGREE;
    }

    public double $getLeftServoPosition() {
        return leftHoodAngler.getPosition();
    }

    public double $getRightServoPosition() {
        return rightHoodAngler.getPosition();
    }

}