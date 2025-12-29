package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BasicVeloMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremeNikhilFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.ExtremePrecisionFlywheel;
import org.firstinspires.ftc.teamcode.ShooterSystems.HoodAngler;
import org.firstinspires.ftc.teamcode.ShooterSystems.Turret;
import org.firstinspires.ftc.teamcode.ShooterSystems.TurretBase;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.roadrunner.CustomMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.AdafruitBeambreakSensor;

import java.util.List;

public class LaNikhilBaseOpMode extends LinearOpMode {

    public volatile BetterGamepad controller1;
    public volatile BetterGamepad controller2;

    public volatile DcMotor left_front, right_front, left_back, right_back;


    public MecanumDrive drive;

    public volatile Turret turret;

    //Put hoodAngling later

    public volatile ExtremeNikhilFlywheel flywheel;

    public Pose2d initialPose;

    public RobotCentricDrive robotCentricDrive;

    public BetterGamepad gamepad1;

    DcMotorEx left_flywheel, right_flywheel;

    Pose2d currentPose;



    public void initializeNikhilDevices() {

        left_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftFrontMotorDeviceName);
        right_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightFrontMotorDeviceName);
        left_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftBackMotorDeviceName);
        right_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightBackMotorDeviceName);


        initialPose =  new Pose2d(0, 0, 0);

        turret = new Turret(hardwareMap, initialPose, 0, 120, gamepad1);
        robotCentricDrive = new RobotCentricDrive();
        //flywheel = new ExtremeNikhilFlywheel(hardwareMap, gamepad1)

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, gamepad1);


    }



    @Override
    public void runOpMode() throws InterruptedException {

    }

}
