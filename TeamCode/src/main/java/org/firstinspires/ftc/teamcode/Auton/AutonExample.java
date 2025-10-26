package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@Autonomous(name="AutonExample")
public class AutonExample extends LinearOpMode {

    Servo shooter;
    public class ShootBall implements InstantFunction {

        float targetPos;

        public ShootBall(float targetPosition){
            this.targetPos = targetPos;
        }

        @Override
        public void run(){
            shooter.setPosition(targetPos);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(Servo.class, "shooter");


        Pose2d beginPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .splineToSplineHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
                .lineToXLinearHeading(0, Math.toRadians(90))
                .stopAndAdd(new ShootBall(0))
                .build();

        Actions.runBlocking(new SequentialAction(path));


       }
}
