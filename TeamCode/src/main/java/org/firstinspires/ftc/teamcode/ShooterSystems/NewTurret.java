package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class NewTurret {

    CRServoImplEx left_turret, right_turret;

    Pose2d initialPose;

    Pose2d currentPose;


    PinpointLocalizer localizer;

    double targetX, targetY;

    Encoder encoder;


    private double kp, ki, kd;

    private double p, i, d;
        double currTime, prevTime, startTime, dT;

        double i_MIN, i_MAX;


    double currError = 0, prevError = 0;

    double errorRate = (currError-prevError) / dT;

    public boolean firstTick = true;
    ElapsedTime timer = new ElapsedTime();


    public NewTurret(HardwareMap hardwareMap, Pose2d initialPose, double targetX, double targetY, DcMotorEx right_back) {
        left_turret = hardwareMap.get(CRServoImplEx.class, "left_turret_base");
        right_turret = hardwareMap.get(CRServoImplEx.class, "right_turret_base");

        this.initialPose = initialPose;

        this.currentPose = initialPose;

        this.targetX = targetX;
        this.targetY = targetY;

        localizer = new PinpointLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, initialPose);

        encoder = new Encoder(right_back);

    }

    public double toTicks(double degrees) {
        return degrees * MecanumDrive.PARAMS.inPerTick;

    }

    public void setPID() {
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
    }

    public double getElapsed() {
        if (firstTick) {
            startTime = timer.seconds();
        }
        return (System.nanoTime() - startTime);
    }

    public void updatePID() {

        p = kp * currError;

        prevTime = currTime;
        currTime = getElapsed();
        dT = currTime - prevTime;

        prevError = currError;

        currError =

        i = MathUtil.clamp(ki, i_MIN, i_MAX);

        if (dT > 0) {
            d = kd * errorRate
        }








    }



    public void update() {

        currentPose = localizer.getPose();

        double RobotHeading = currentPose.heading.toDouble();

        double currX = currentPose.position.x;
        double currY = currentPose.position.y;

        double dX = currX - targetX;
        double dY = currY - targetY;

        double FieldAngle = FastMath.atan2(dY, dX);

        double turretCurrPos = encoder.getCurrentPosition();
        double robotTurn = FastMath.abs(FieldAngle - RobotHeading);
        double turretOffset = encoder.getCurrentPosition();

        double turretTurnDegrees = (180- robotTurn) - turretOffset;

        double turnTicks = toTicks(turretTurnDegrees);


    }




}
