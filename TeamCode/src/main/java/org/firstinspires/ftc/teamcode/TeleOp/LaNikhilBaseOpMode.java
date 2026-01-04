package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremeNikhilFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.Turret;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class LaNikhilBaseOpMode extends LinearOpMode {

    public volatile BetterGamepad gamepad1;
    public volatile BetterGamepad gamepad2;

    public volatile DcMotor left_front, right_front, left_back, right_back;


    public MecanumDrive drive;

    public volatile Turret turret;

    //Put hoodAngling later

    public volatile ExtremeNikhilFlywheel flywheel;

    public Pose2d initialPose;

    public RobotCentricDrive robotCentricDrive;

    DcMotorEx left_flywheel, right_flywheel;

    Pose2d currentPose;


    Encoder encoder;



    public void initializeNikhilDevices(DcMotor left_front, DcMotor right_front, DcMotor left_back, DcMotor right_back) {

        left_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftFrontMotorDeviceName);
        right_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightFrontMotorDeviceName);
        left_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftBackMotorDeviceName);
        right_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightBackMotorDeviceName);

        initialPose =  new Pose2d(-60, 0, 0);

        turret = new Turret(hardwareMap, initialPose, 0, 120);
        robotCentricDrive = new RobotCentricDrive();
        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, gamepad1);
        flywheel = new ExtremeNikhilFlywheel(hardwareMap, initialPose);


    }



    public void runOpMode() throws InterruptedException {

    }

}
