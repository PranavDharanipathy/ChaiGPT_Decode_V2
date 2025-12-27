package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;

@TeleOp(name = "Learning Turret Aiming")
public class TurretAimTest extends OpMode {

    Turret turret;

    Pose2d initialPose;

    RobotCentricDrive robotCentricDrive = new RobotCentricDrive();


    DcMotor left_front, left_back, right_front, right_back;
    BetterGamepad controller1;




    @Override
    public void init() {

         initialPose=  new Pose2d(0, 0, -45);

        turret = new Turret(hardwareMap, initialPose, 0, 120);

        robotCentricDrive.provideComponents(left_front, right_front, left_back, right_back, controller1);




    }

    @Override
    public void loop() {

        turret.update();
        robotCentricDrive.update();












    }
}
