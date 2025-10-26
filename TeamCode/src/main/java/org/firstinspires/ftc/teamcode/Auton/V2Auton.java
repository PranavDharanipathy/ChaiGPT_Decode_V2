package org.firstinspires.ftc.teamcode.Auton;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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

    CommandScheduler scheduler = new CommandScheduler();


    public void runOpMode() {

        //example: drive forward, wait one second(not obstructing drive forward), and then drive back.

        //first motor --> drive forward

        //wait 1 sec
        scheduler.add(new WaitCommand(1000));
        //second motor

    }



}
