package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.TeleOp.LaNikhilBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Learning Turret Aiming")
public class TurretAimTest extends LaNikhilBaseOpMode {

    //HardwareMap hardwareMap;

    public volatile BetterGamepad gamepad1;
    public volatile BetterGamepad gamepad2;

    public volatile DcMotor left_front, right_front, left_back, right_back;


    public MecanumDrive drive;

   // public HardwareMap hardwareMap;

    public volatile Turret turret;

    //Put hoodAngling later

    public volatile ExtremeNikhilFlywheel flywheel;

    public Pose2d initialPose;

    public RobotCentricDrive robotCentricDrive;

    DcMotorEx left_flywheel, right_flywheel;

    Pose2d currentPose;

    @Override
    public void runOpMode() {


        initializeNikhilDevices();


        while (opModeIsActive() && !isStopRequested()) {

            turret.update();
            robotCentricDrive.update();
            flywheel.update();

        }


    }


}


