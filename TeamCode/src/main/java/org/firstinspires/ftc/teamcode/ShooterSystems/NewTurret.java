package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.roadrunner.Pose2d;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class NewTurret {

    CRServoImplEx left_turret, right_turret;

    Pose initialPose;

    Pose currentPose;


    PinpointLocalizer localizer;

    double targetX, targetY;

    Encoder encoder;


    private double kp, ki, kd;

    private double p = 0, i = 0, d = 0;
        double currTime = 0, prevTime = 0, startTime = 0, dT = 0;

        double i_MIN = 0, i_MAX = 0;


    double currError = 0, prevError = 0;

    double errorRate = 0;

    public boolean firstTick = true;
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx right_back;

    double RobotHeading ;

    double currX ;
    double currY ;

    double dX;
    double dY;

    double FieldAngle;

    double turretCurrPos;
    double robotTurn;
    double turretOffset;

    double turretTurnDegrees ;

    double turnTicks ;

    double power;


    public NewTurret(HardwareMap hardwareMap, Pose initialPose, double targetX, double targetY) {

        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        left_turret = hardwareMap.get(CRServoImplEx.class, "left_turret_base");
        right_turret = hardwareMap.get(CRServoImplEx.class, "right_turret_base");

        this.initialPose = initialPose;

        this.currentPose = initialPose;

        this.targetX = targetX;
        this.targetY = targetY;

        localizer = new PinpointLocalizer(hardwareMap, PPConstants.localizerConstants, initialPose);

        encoder = new Encoder(right_back);

    }

    public double toTicks(double degrees) {
        return degrees * MecanumDrive.PARAMS.inPerTick;

    }

    public void setPID(double kp, double ki, double kd) {
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

         errorRate = (currError-prevError) / dT;

        p = kp * currError;

        prevTime = currTime;
        currTime = getElapsed();
        dT = currTime - prevTime;

        prevError = currError;

        currError = turretCurrPos - turnTicks;

        i = MathUtil.clamp(ki, i_MIN, i_MAX);

        if (dT > 0) {
            d = kd * errorRate;
        }

        power = p + i + d;

        left_turret.setPower(power);
        right_turret.setPower(power);





    }



    public void update() {

        currX = localizer.getPose().getX();

        currY = localizer.getPose().getY();


        dX = targetX - currX;
        dY = targetY - currY;



        updatePID();


    }




}
