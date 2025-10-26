package org.firstinspires.ftc.teamcode.Auton;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "V2Auton")
public class V2Auton {

    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

    CommandScheduler scheduler = new CommandScheduler();


    public void runOpMode() {

        //example: drive forward, wait one second(not obstructing drive forward), and then drive back.

        //first motor --> drive forward

        drive.setDrivePowers();
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //wait 1 sec
        scheduler.add(new WaitCommand(1000));
        //second motor

    }



}
