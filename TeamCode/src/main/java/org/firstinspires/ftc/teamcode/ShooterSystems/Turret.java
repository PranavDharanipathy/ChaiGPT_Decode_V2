package org.firstinspires.ftc.teamcode.ShooterSystems;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.Objects;

@Config
public class Turret {

/*

        //f = fAdjuster * realKf * (reZeroedTargetPosition - lanyardEquilibrium);

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

   public static double currX, currY;
   double dX, dY;
   double desiredFieldAngleDeg;

   double currentRobotHeading;

   double turretTargetDegrees;
   public double turretTargetTicks;


    PinpointLocalizer localizer;

    Pose2d initialPose, currentPose;

    double targetX, targetY;

    CRServoImplEx left_turret, right_turret;

    Encoder encoder;

    public double turretCurrPosTicks;

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

    private double startTime = 0;

    public double getElapsed() {
        return (System.nanoTime() * 1e-9) - startTime;
    }

    public double
            kp = 0
            , ki = 0
            , kd = 0
            , p = 0
            , i = 0
            , d = 0
            , error = 0
            , prevError = 0
            , errorSum = 0
            , prevTime
            , currTime
            , dt = 0
            , imax, imin;

    public void setPIDFCoefficients(double kp, double ki, double kd) {

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

    }

    private boolean firstTick = true;

    public void updatePID() {

        if (firstTick) {

            startTime = getElapsed();

            firstTick = false;
        }

        dt = currTime - prevTime;

        if (dt == 0) return;

        prevError = error;
        error = turretTargetTicks - encoder.getCurrentPosition();

        //p
        p = kp * error;

        //i
        errorSum += error * dt;
        i = MathUtil.clamp(ki * errorSum, imin, imax);

        //d
        final double tempD = kd * (error - prevError) / dt;
        d = !Double.isNaN(tempD) ? tempD : 0;

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
        turretTargetTicks = (turretTargetDegrees * 73.5179487179) + Math.abs( turretStartPos);


        //set turret to desired location
        //left_turret.setPosition(turretTargetTicks);


        turretCurrPosTicks = encoder.getCurrentPosition();


        //right_turret.setPosition(turretTargetTicks);
        //Extra features: rumbling(like shooting)

        //If turret is out of bounds


        updatePID();
    }


}

