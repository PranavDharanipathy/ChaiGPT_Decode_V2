package org.firstinspires.ftc.teamcode.ShooterSystems;

import android.graphics.LinearGradient;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.util.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.TeleOp.LaNikhilBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Learning Turret Aiming")
public class TurretAimTest extends LinearOpMode {

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

    Gamepad controller1 = new Gamepad();

    Gamepad controller2 = new Gamepad();






    @Override
    public void runOpMode() {


        left_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftFrontMotorDeviceName);
        right_front = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightFrontMotorDeviceName);
        left_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.leftBackMotorDeviceName);
        right_back = hardwareMap.get(DcMotor.class, Constants.MapSetterConstants.rightBackMotorDeviceName);


        this.gamepad1 = new BetterGamepad(controller1);
        this.gamepad2 = new BetterGamepad(controller2);


        initialPose = new Pose2d(62, 9, Math.toRadians(90));

        turret = new Turret(hardwareMap, initialPose, 0, 120);
        robotCentricDrive = new RobotCentricDrive();
        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, gamepad1);
        flywheel = new ExtremeNikhilFlywheel(hardwareMap, initialPose);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            turret.update();
            robotCentricDrive.update();
            flywheel.update();

        }

        if (isStopRequested()) {
            telemetry.addLine("ended");
        }

    }


    @Override
    public void waitForStart() {
        super.waitForStart();
    }
}


