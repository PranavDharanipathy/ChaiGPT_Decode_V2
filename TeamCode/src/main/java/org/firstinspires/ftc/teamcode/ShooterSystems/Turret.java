package org.firstinspires.ftc.teamcode.ShooterSystems;

import static org.apache.commons.math3.util.FastMath.toDegrees;

import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Turret {

/*

        f = fAdjuster * realKf * (reZeroedTargetPosition - lanyardEquilibrium);

        //static friction feedforward
        s = ks * Math.signum(error);

        i = MathUtil.clamp(i, MIN_I, MAX_I);

        d = dt > 0 ? kd * filteredDerivative : 0;

                double currentPosition = getCurrentPosition();

        currTime = timer.milliseconds();
        double dt = currTime - prevTime;

        error = targetPosition - currentPosition;

        //proportional
        p = kp * error;
 */

   public double currX, currY;
   double dX, dY;
   double desiredFieldAngleDeg;

   double currentRobotHeading;

   double turretTargetDegrees, turretTargetTicks;


    PinpointLocalizer localizer;

    Pose2d initialPose, currentPose;

    double targetX, targetY;

    CRServoImplEx left_turret, right_turret;

    Encoder encoder;

    double turretCurrPosTicks;

    BetterGamepad gamepad1;

    double turretCurrDeg;
    double turretStartPos;

    Gamepad controller1 = new Gamepad();

    public Turret( HardwareMap hardwareMap, Pose2d initialPose, double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;

        this.gamepad1 = new BetterGamepad(controller1);

        this.initialPose = initialPose;

        left_turret = hardwareMap.get( CRServoImplEx.class, "left_turret_base");
        right_turret = hardwareMap.get(CRServoImplEx.class, "right_turret_base");

        //use right_back motor since encoder is plugged into that
        encoder = new Encoder(hardwareMap.get(DcMotorEx.class, Constants.MapSetterConstants.turretExternalEncoderMotorPairName));
        //set encoder direction to the same direction as the right_back motor
        encoder.setDirection(Encoder.Direction.FORWARD);
        localizer = new PinpointLocalizer(hardwareMap, 73.5179487179, initialPose);

        turretStartPos = encoder.getCurrentPosition();

    }

    public void reverse() {

        DcMotorSimple.Direction direction = left_turret.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;

        left_turret.setDirection(direction);
        right_turret.setDirection(direction);
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
        currentRobotHeading = toDegrees(currentPose.heading.toDouble());
        turretTargetDegrees = desiredFieldAngleDeg - currentRobotHeading;

        //convert degrees to ticks
        turretTargetTicks = (turretTargetDegrees * 73.5179487179) + turretStartPos;


        //set turret to desired location
        //left_turret.setPosition(turretTargetTicks);

        turretCurrPosTicks = encoder.getCurrentPosition();
        double error = turretTargetTicks - turretCurrPosTicks;

        double kp = 0.00045;
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
