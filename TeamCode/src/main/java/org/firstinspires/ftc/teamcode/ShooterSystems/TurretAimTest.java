package org.firstinspires.ftc.teamcode.ShooterSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED.BetterGamepad;
import org.firstinspires.ftc.teamcode.TeleOp.LaNikhilBaseOpMode;
import org.firstinspires.ftc.teamcode.TeleOp.drive.RobotCentricDrive;

@TeleOp(name = "Learning Turret Aiming")
public class TurretAimTest extends LaNikhilBaseOpMode {

    //HardwareMap hardwareMap;


    @Override
    public void runOpMode() {


        while (opModeIsActive() && !isStopRequested()) {
            turret.update();
            robotCentricDrive.update();
            flywheel.update();

        }














    }
}
