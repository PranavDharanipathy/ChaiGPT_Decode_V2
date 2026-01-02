package org.firstinspires.ftc.teamcode.ShooterSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.opmodecontrol.ActiveOpMode;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Objects;

public final class ExtremeNikhilFlywheel {

    //Parameters in constructor
    DcMotorEx left_flywheel; //has encoder on it
    DcMotorEx right_flywheel;  //on the same shaft as left_flywheel

    Encoder encoder;
    Pose2d currentPose;

    Pose2d initialPose;

    //CustomMecanumDrive localizer;

    PinpointLocalizer localizer;

    public ExtremeNikhilFlywheel(HardwareMap hardwareMap, DcMotorEx left_flywheel, DcMotorEx right_flywheel, Pose2d initialPose, Encoder encoder) {

        this.left_flywheel = left_flywheel;
        this.right_flywheel = right_flywheel;

        this.encoder = encoder;

        this.initialPose = initialPose;

        localizer = new PinpointLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, initialPose);

        //encoder is only on left_flywheel

        encoder = new Encoder(left_flywheel);

        //set encoder direction to same direction as flywheel

        encoder.setDirection(Encoder.Direction.REVERSE);

        left_flywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[0]);
        right_flywheel.setDirection(Constants.FLYWHEEL_MOTOR_DIRECTIONS[1]);

        left_flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right_flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        left_flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    double velocity;


    public void setVelocity(double velocity) {

        this.velocity = velocity;

        left_flywheel.setVelocity(velocity);

        right_flywheel.setVelocity(velocity);
    }

    public void stop() {

        left_flywheel.setPower(0);
        right_flywheel.setPower(0);
    }


    public void update() {


        currentPose = localizer.getPose();

        if (currentPose.position.x > ShooterInformation.ShooterConstants.CLOSE_SIDE_SWITCH) {

            setVelocity(ShooterInformation.ShooterConstants.FARTHER_CLOSE_SIDE_FLYWHEEL_SHOOT_VELOCITY);
        }
        else if (currentPose.position.x < ShooterInformation.ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER) {
            setVelocity(ShooterInformation.ShooterConstants.FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY);
        }
        else {
            setVelocity(410_000);
        }




    }

    //END
}
