package org.firstinspires.ftc.teamcode.ShooterSystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static java.lang.Double.NaN;

import com.acmerobotics.roadrunner.Pose2d;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.HubAddressChangedNotification;
import org.firstinspires.ftc.teamcode.pedroPathing.PPConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Rev9AxisImuWrapped;


public class NewTurret {

    CRServoImplEx left_turret, right_turret;

    Pose initialPose;

    Pose currentPose;


    PinpointLocalizer localizer;

    double targetX, targetY;

    Encoder encoder;


    private double kp = 0, ki = 0, kd = 0;

    private double p = 0, i = 0, d = 0;
        double currTime = 0, prevTime = 0, startTime = 0, dT = 0;

        double i_MIN = 0, i_MAX = 0;


    double currError = 0, prevError = 0;

    double errorRate = 0;

    public boolean firstTick = true;
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx right_back;

    double currX = 12;
    double currY =12;

    double dX = 0;
    double dY = 0;

    double FieldAngle;

    double turretCurrPos = 0;
    double robotTurn = 0;
    double turretOffset = 0;

    double turretTurnTicks = 0;

    double turnTicks = 0;

    double power = 0;

    Follower follower;

    double fieldAngle = 0;

    double robotHeading = 0;



    public NewTurret(HardwareMap hardwareMap, Pose initialPose, double targetX, double targetY) {

        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        left_turret = hardwareMap.get(CRServoImplEx.class, "left_turret_base");
        right_turret = hardwareMap.get(CRServoImplEx.class, "right_turret_base");

        this.initialPose = initialPose;

        this.currentPose = initialPose;

        this.targetX = targetX;
        this.targetY = targetY;

        follower = PPConstants.createAutoFollower(hardwareMap);

        localizer = new PinpointLocalizer(hardwareMap, PPConstants.localizerConstants, initialPose);

        encoder = new Encoder(right_back);

        follower.setStartingPose(initialPose);

        follower.update();
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
        return (System.nanoTime() * 1e-9 - startTime);
    }

    public void updatePID() {

        if (!(dT == 0)) {
            errorRate = (currError-prevError) / dT;
        }

        p = kp * currError;

        prevTime = currTime;
        currTime = getElapsed();
        dT = currTime - prevTime;

        prevError = currError;
        currError = turretTurnTicks - turretCurrPos;

        i = MathUtil.clamp(ki, i_MIN, i_MAX);

        if (dT > 0) {
            d = kd * errorRate;
        }

        power = MathUtil.clamp((p + i + d), -1, 1);

        left_turret.setPower(power);
        right_turret.setPower(power);
    }
    double toTicks(double degrees) {
        return degrees * MecanumDrive.PARAMS.inPerTick;
    }



    public void update() {


        currentPose = follower.getPose();

        currX = currentPose.getX();
        currY = currentPose.getY();

        turretCurrPos = encoder.getCurrentPosition();

        dX = targetX - currX;
        dY = targetY - currY;

        fieldAngle = FastMath.atan2(dY,dX);

        robotHeading = currentPose.getHeading();


        robotTurn = 45 + (FastMath.abs(fieldAngle - robotHeading));

        updatePID();
    }
}
