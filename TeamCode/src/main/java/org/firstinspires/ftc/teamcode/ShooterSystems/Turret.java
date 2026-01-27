package org.firstinspires.ftc.teamcode.ShooterSystems;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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

   public volatile double currX, currY;
   public volatile double dX, dY;
    public volatile double desiredFieldAngleDeg;

   public volatile double currentRobotHeading;

   public volatile double turretTargetDegrees;
   public double turnticks;


    public volatile PinpointLocalizer localizer;

    public volatile Pose2d initialPose, currentPose;

    double targetX, targetY;

    CRServoImplEx left_turret, right_turret;

    Encoder encoder;

    public volatile double turretCurrPosTicks;

    BetterGamepad gamepad1;

    volatile double turretCurrDeg;
    public volatile double turretStartPos;

    Gamepad controller1 = new Gamepad();

    public volatile double turretTargetTicks;

    public double power;

    double turretTurnDegrees;

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


    }

    private double wrapDegrees(double deg) {
        // result will be in [-180.0, 180.0)
        double r = ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return r;
    }



    public void update() {

        localizer.update();
        currentPose = localizer.getPose();

        currX =  currentPose.position.x;
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
        turretTurnDegrees = 180 - turretTargetDegrees;

        turnticks = (turretTurnDegrees* 73.5179487179);

        turretCurrPosTicks = encoder.getCurrentPosition();

//PID

        if (firstTick) {

            startTime = getElapsed();

            firstTick = false;
        }

        currTime = System.nanoTime() * 1e-9;

        dt = currTime - prevTime;
        prevTime = currTime;

        if (dt == 0) return;

        prevError = error;
        error = turnticks - encoder.getCurrentPosition();

        //p
        p = kp * error;

        //i
        errorSum += error * dt;
        i = MathUtil.clamp(ki * errorSum, imin, imax);

        //d
        final double tempD = kd * (error - prevError) / dt;
        d = !Double.isNaN(tempD) ? tempD : 0;

        power = p + i+d;


    }


}

