package org.firstinspires.ftc.teamcode.ShooterSystems;

import static org.apache.commons.math3.util.FastMath.toDegrees;

import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Turret {

       public double currX;
       double currY;
       double dX;
       double dY;
       double desiredFieldAngleDeg;

       double currentRobotHeading;

       double turretTargetDegrees;
       double turretTargetTicks;


    PinpointLocalizer localizer;

    Pose2d initialPose;

    Pose2d currentPose;

    double targetX;

    double targetY;

    CRServo left_turret;

    CRServo right_turret;

    Encoder encoder;

    double turretCurrPosTicks;

    BetterGamepad gamepad1;

    double turretCurrDeg;

    Gamepad controller1 = new Gamepad();

    public Turret( HardwareMap hardwareMap, Pose2d initialPose, double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;

        this.gamepad1 = new BetterGamepad(controller1);

        this.initialPose = initialPose;

        left_turret = hardwareMap.get( CRServo.class, "left_turret_base");
        right_turret = hardwareMap.get(CRServo.class, "right_turret_base");
        DcMotorEx rb = hardwareMap.get(DcMotorEx.class, "right_back");

        //use right_back motor since encoder is plugged into that
        encoder = new Encoder(rb);
        //set encoder direction to the same direction as the right_back motor
        encoder.setDirection(Encoder.Direction.REVERSE);
        localizer = new PinpointLocalizer(hardwareMap, 73.5179487179, initialPose);

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
        //left_turret.setPosition(turretTargetTicks);

        turretCurrPosTicks = encoder.getCurrentPosition();
        double error = turretTargetTicks - turretCurrPosTicks;

        double kp = 0.0015;
        double power = kp * error;


        left_turret.setPower(power);
        right_turret.setPower(power);

        if (Math.abs(error) < 10) {
            left_turret.setPower(0);
            right_turret.setPower(0);
        }



        //right_turret.setPosition(turretTargetTicks);
        //Extra features: rumbling(like shooting)

        //If turret is out of bounds


    }
}
