package org.firstinspires.ftc.teamcode.ShooterSystems;

import static com.sun.tools.doclint.Entity.aacute;
import static com.sun.tools.doclint.Entity.lambda;
import static org.apache.commons.math3.util.FastMath.toDegrees;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;

public class Turret {

       public double currX;
       double currY;
       double dX;
       double dY;

       double desiredFieldAngleRad;

       double desiredFieldAngleDeg;

       double currentRobotHeading;

       double turretTargetDegrees;
       double turretTargetTicks;


    ThreeDeadWheelLocalizer localizer;

    Pose2d initialPose;

    Pose2d currentPose;

    double targetX;

    double targetY;

    Servo turret;

    double turretCurrPos;

    BetterGamepad gamepad1;

    double turretCurrDeg;


    public Turret(HardwareMap hardwareMap, Pose2d initialPose, double targetX, double targetY, BetterGamepad gamepad1) {
        this.targetX = targetX;
        this.targetY = targetY;

        this.gamepad1 = gamepad1;

        this.initialPose = initialPose;

        turret = hardwareMap.get(Servo.class, "turret");


        //Initializing Devices(Pose and Localizer to be used in loop();

        //initialPose = new Pose2d(-11, 23.5, 0);

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, 0.0020446994, initialPose);

    }



    public void update() {

        localizer.update();

        currentPose = localizer.getPose();

        currX = currentPose.position.x;

        currY = currentPose.position.y;

        dX = targetX - currX;

        dY = targetY - currY;


        //atan2 returns radians --> Angle of slope between distance

        //convert desiredFieldAngle to degrees with toDegrees()

        desiredFieldAngleDeg = toDegrees(FastMath.atan2(dY, dX));

        //in degrees?

        currentRobotHeading = currentPose.heading.toDouble();

        turretTargetDegrees = desiredFieldAngleDeg - currentRobotHeading;

        //convert degrees to ticks

        turretTargetTicks = turretTargetDegrees * 73.5179487179;


        //set turret to desired location
        turret.setPosition(turretTargetTicks);


        //Extra features: rumbling(like shooting)

        turretCurrPos = turret.getPosition();

        turretCurrDeg = toDegrees(turretCurrPos);


        if (turretCurrPos == turretTargetTicks) { gamepad1.rumble(7000); }
        else {gamepad1.stopRumble(); }



        //If turret is out of bounds
        if (turretCurrDeg > 176 || turretCurrDeg < -170) {
            turret.setPosition(0);
        }


    }
}
