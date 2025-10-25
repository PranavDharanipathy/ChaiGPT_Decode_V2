package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.chaigptrobotics.shenanigans.Peak;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Peak
/// Uses AXONs
public class HoodAngler {

    private Servo leftHoodAngler;
    private Servo rightHoodAngler;

    public HoodAngler(HardwareMap hardwareMap, String leftHoodAnglerServoName, String rightHoodAnglerServoName) {

        leftHoodAngler = hardwareMap.get(Servo.class, leftHoodAnglerServoName);
        rightHoodAngler = hardwareMap.get(Servo.class, rightHoodAnglerServoName);
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

    public void setPosition(double position) {

        leftHoodAngler.setPosition(position);
        rightHoodAngler.setPosition(position);
    }

    public double getPosition() {
        return position;
    }
}