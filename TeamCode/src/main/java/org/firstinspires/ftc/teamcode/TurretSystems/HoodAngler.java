package org.firstinspires.ftc.teamcode.TurretSystems;

import org.firstinspires.ftc.teamcode.TurretSystems.ShooterInformation.ShooterConstants;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HoodAngler {

    private ServoImplEx leftHoodAngler;
    private ServoImplEx rightHoodAngler;

    private HoodAngler(ServoImplEx leftHoodAnglerServo, ServoImplEx rightHoodAnglerServo) {

        leftHoodAngler = leftHoodAnglerServo;
        rightHoodAngler = rightHoodAnglerServo;

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

    /// @param angle - is in degrees
    public void setAngle(double angle) {

        double position = (angle + ShooterConstants.TURRET_DEGREES_PER_HOOD_ANGLER_DEGREE) * ShooterConstants.HOOD_ANGLER_POSITIONAL_INCREMENT_PER_DEGREE;

        leftHoodAngler.setPosition(position);
        rightHoodAngler.setPosition(position);
    }



}
