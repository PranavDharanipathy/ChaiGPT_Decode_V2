package org.firstinspires.ftc.teamcode.ShooterSystems;

import static com.sun.tools.doclint.Entity.aacute;
import static com.sun.tools.doclint.Entity.lambda;
import static com.sun.tools.javac.jvm.ByteCodes.error;
import static org.apache.commons.math3.util.FastMath.toDegrees;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

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


    MecanumDrive MecanumDrive;

    Pose2d initialPose;

    Pose2d currentPose;

    double targetX;

    double targetY;

    CRServo left_turret;

    CRServo right_turret;

    Encoder encoder;

    double turretCurrPos;

    BetterGamepad gamepad1;

    double turretCurrDeg;

    public Turret(HardwareMap hardwareMap, Pose2d initialPose, double targetX, double targetY, BetterGamepad gamepad1) {
        this.targetX = targetX;
        this.targetY = targetY;

        this.gamepad1 = gamepad1;

        this.initialPose = initialPose;

        left_turret = hardwareMap.get( CRServo.class, "left_turret");

        right_turret = hardwareMap.get(CRServo.class, "right_turret");
        encoder = new Encoder(left_turret);

        MecanumDrive = new MecanumDrive(hardwareMap, initialPose);

    }

    public void setPosition(double target){
        double p = 0.017;



        double error = target - turretCurrPos;

        double power = p * error;

        left_turret.setPower(power);
        right_turret.setPower(power);


    }



    public void update() {


        MecanumDrive.localizer.update();

        currentPose = MecanumDrive.localizer.getPose();

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
        //left_turret.setPosition(turretTargetTicks);


        setPosition(turretTargetTicks);


        //right_turret.setPosition(turretTargetTicksc);


        //Extra features: rumbling(like shooting)

        turretCurrPos = encoder.getCurrentPosition();

        turretCurrDeg = turretCurrPos/73.5179487179;


        if (turretCurrPos == turretTargetTicks) { gamepad1.rumble(7000); }
        else {gamepad1.stopRumble(); }



        //If turret is out of bounds
        if (turretCurrDeg > 176 || turretCurrDeg < -170) {
            left_turret.setPower(0);
            right_turret.setPower(0);
        }


    }
}
