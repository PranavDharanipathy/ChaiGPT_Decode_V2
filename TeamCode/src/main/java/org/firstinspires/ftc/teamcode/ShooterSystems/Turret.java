package org.firstinspires.ftc.teamcode.ShooterSystems;

import static org.apache.commons.math3.util.FastMath.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
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

    Gamepad controller2;

    public Turret( HardwareMap hardwareMap, Pose2d initialPose, double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;

        this.gamepad1 = new BetterGamepad(controller1);



        this.initialPose = initialPose;

        left_turret = hardwareMap.get( CRServo.class, "left_turret");

        right_turret = hardwareMap.get(CRServo.class, "right_turret");
        DcMotorEx rb = hardwareMap.get(DcMotorEx.class, "right_back");
        //use right_back motor since encoder is plugged into that
        encoder = new Encoder(rb);
        //set encoder direction to the same direction as the right_back motor
        encoder.setDirection(Encoder.Direction.REVERSE);

        localizer = new PinpointLocalizer(hardwareMap, 73.5179487179, initialPose);

    }

    public void setPosition(double target){
        double p = 0.017;
        double error = target - turretCurrPosTicks;

        double power = p * error;
        left_turret.setPower(power);
        right_turret.setPower(power);

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

        setPosition(turretTargetTicks);

        //right_turret.setPosition(turretTargetTicksc);
        //Extra features: rumbling(like shooting)

        turretCurrPosTicks = encoder.getCurrentPosition();
        turretCurrDeg = turretCurrPosTicks / 73.5179487179;


        if (turretCurrPosTicks == turretTargetTicks) { gamepad1.rumble(2000);  }
        else {gamepad1.stopRumble(); }


        //If turret is out of bounds
        if (turretCurrDeg > 176 || turretCurrDeg < -170) {
            left_turret.setPower(0);
            right_turret.setPower(0);
        }


    }
}
