package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodTest {

    Servo left_hood;
    Servo right_hood;

    public HoodTest(HardwareMap hardwareMap) {

        left_hood = hardwareMap.get(Servo.class, "left_hood");
        right_hood = hardwareMap.get(Servo.class, "right_hood");
    }

    public void setHoodAnglerDirections() {

        left_hood.setDirection(Servo.Direction.FORWARD);
        right_hood.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(double position) {
        left_hood.setPosition(position);
        right_hood.setPosition(position);
    }

    public void resetAngles() {
        left_hood.setPosition(0.11);
        right_hood.setPosition(0.11);
    }





}
